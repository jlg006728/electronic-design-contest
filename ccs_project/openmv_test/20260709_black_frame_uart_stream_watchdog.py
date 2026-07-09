# OpenMV active UART stream for MSPM0 dual-servo tracking.
# 20260709_2150:
#   Black-frame target-center version.
#   1. Keep the existing UART frame protocol, watchdog, and recovery loop.
#   2. Replace red/ring target detection with black outer-frame detection.
#   3. Prefer find_rects() when available, then fall back to merged black blobs.
#   4. Report the averaged four-corner center as cx/cy.
#
# 20260705_145746:
#   1. Keep streaming color frames without waiting for MSPM0 commands.
#   2. Never print during standalone operation.
#   3. Drain UART RX with a hard byte limit so RX noise cannot starve vision.
#   4. Catch Python-level camera/blob/UART exceptions and reinitialize camera.
#   5. Enable watchdog when available so a firmware-level hang can auto-reset.
#   6. Reject blobs touching the image border so MSPM0 searches instead of
#      chasing a partial red target at the edge of the frame.
#
# Wiring:
#   OpenMV P4 / UART3_TX -> MSPM0 PA11 / UART0_RX
#   OpenMV P5 / UART3_RX <- MSPM0 PA10 / UART0_TX
#   OpenMV GND           -> MSPM0 GND
#
# Reply frame, 17 bytes:
#   0: 0xAA
#   1: 0x55
#   2: type      1=P, 2=S, 3=C
#   3: seq low
#   4: seq high
#   5: flags     bit0=camera_ok, bit1=target_detected, bit2=uart_rx_seen
#   6: cx low    int16, -1 when no target
#   7: cx high
#   8: cy low
#   9: cy high
#   10: pixels low
#   11: pixels high
#   12: fps_x10 low
#   13: fps_x10 high
#   14: l_mean   0..255 approximate scene brightness
#   15: checksum XOR of bytes 2..14
#   16: 0x5B

import gc
import sensor
import time
from pyb import LED, UART, millis

try:
    from pyb import WDT
except Exception:
    WDT = None


UART_BAUD = 115200
UART_STREAM_INTERVAL_MS = 25
UART_DRAIN_MAX_BYTES = 16
GC_INTERVAL_FRAMES = 80

ENABLE_DEBUG_PRINT = True
ENABLE_WATCHDOG = True
WATCHDOG_TIMEOUT_MS = 3000

FRAME_HEADER0 = 0xAA
FRAME_HEADER1 = 0x55
FRAME_FOOTER = 0x5B

CMD_PING = ord("P")
CMD_STATS = ord("S")
CMD_COLOR = ord("C")

TYPE_PING = 1
TYPE_STATS = 2
TYPE_COLOR = 3

FLAG_CAMERA_OK = 0x01
FLAG_TARGET_DETECTED = 0x02
FLAG_UART_RX_SEEN = 0x04

IMG_W = 320
IMG_H = 240
IMG_CENTER_X = IMG_W // 2
IMG_CENTER_Y = IMG_H // 2

# Current mechanical installation is upside down.
CAMERA_HMIRROR = True
CAMERA_VFLIP = True

# RGB565 LAB threshold for black/dark tape:
# (L min, L max, A min, A max, B min, B max).
BLACK_THRESHOLDS = [
    (0, 45, -30, 30, -30, 30),
    (0, 70, -45, 45, -45, 45),
]

RECT_THRESHOLD = 6000
BLACK_MIN_PIXELS = 4
BLACK_MIN_AREA = 4
BLACK_MERGE_MARGIN = 30
BLACK_GROUP_MIN_PARTS = 2
BLACK_GROUP_MAX_SINGLE_W = 95
BLACK_GROUP_MAX_SINGLE_H = 95

BORDER_REJECT_MARGIN = 6
MIN_BOX_W = 10
MIN_BOX_H = 10
MAX_BOX_W = 220
MAX_BOX_H = 190
MIN_ASPECT_X100 = 35
MAX_ASPECT_X100 = 285
MIN_DENSITY_PERCENT = 1
MAX_DENSITY_PERCENT = 90
TARGET_HOLD_FRAMES = 8

TARGET_METHOD_RECT = 1
TARGET_METHOD_BLOB = 2
TARGET_METHOD_GROUP = 3


uart = UART(3, UART_BAUD, timeout_char=20)
clock = time.clock()

led_red = LED(1)
led_green = LED(2)
led_blue = LED(3)

g_seq = 0
g_uart_rx_seen = False
g_last_print_ms = 0
g_last_led_ms = 0
g_last_stream_ms = 0
g_loop_count = 0
g_exception_count = 0
g_wdt = None
g_last_target = None
g_target_hold_frames = 0


def clamp_u16(value):
    if value < 0:
        return 0
    if value > 65535:
        return 65535
    return int(value)


def encode_s16(value):
    if value < 0:
        return 0xFFFF
    return clamp_u16(value)


def checksum(payload):
    value = 0
    for b in payload:
        value ^= b
    return value & 0xFF


def make_frame(frame_type, flags, cx, cy, pixels, fps_x10, l_mean):
    global g_seq

    cx_u16 = encode_s16(cx)
    cy_u16 = encode_s16(cy)
    pixels_u16 = clamp_u16(pixels)
    fps_u16 = clamp_u16(fps_x10)
    l_u8 = max(0, min(255, int(l_mean)))

    payload = bytearray([
        frame_type & 0xFF,
        g_seq & 0xFF,
        (g_seq >> 8) & 0xFF,
        flags & 0xFF,
        cx_u16 & 0xFF,
        (cx_u16 >> 8) & 0xFF,
        cy_u16 & 0xFF,
        (cy_u16 >> 8) & 0xFF,
        pixels_u16 & 0xFF,
        (pixels_u16 >> 8) & 0xFF,
        fps_u16 & 0xFF,
        (fps_u16 >> 8) & 0xFF,
        l_u8,
    ])

    frame = bytearray([FRAME_HEADER0, FRAME_HEADER1]) + payload
    frame.append(checksum(payload))
    frame.append(FRAME_FOOTER)
    g_seq = (g_seq + 1) & 0xFFFF
    return frame


def scene_l_mean(img):
    try:
        return int(img.get_statistics().l_mean())
    except Exception:
        return 0


def bbox_corners(x, y, w, h):
    return (
        (x, y),
        (x + w, y),
        (x + w, y + h),
        (x, y + h),
    )


def average_center(corners):
    total_x = 0
    total_y = 0
    count = 0
    for x, y in corners:
        total_x += x
        total_y += y
        count += 1
    if count == 0:
        return -1, -1
    return total_x // count, total_y // count


def density_percent(pixels, w, h):
    area = w * h
    if area <= 0:
        return 0
    return (pixels * 100) // area


def aspect_x100(w, h):
    if h <= 0:
        return 999
    return (w * 100) // h


def is_border_bbox(x, y, w, h):
    if x <= BORDER_REJECT_MARGIN:
        return True
    if y <= BORDER_REJECT_MARGIN:
        return True
    if x + w >= IMG_W - BORDER_REJECT_MARGIN:
        return True
    if y + h >= IMG_H - BORDER_REJECT_MARGIN:
        return True
    return False


def classify_bbox(x, y, w, h, density):
    if is_border_bbox(x, y, w, h):
        return False

    if w < MIN_BOX_W or h < MIN_BOX_H or w > MAX_BOX_W or h > MAX_BOX_H:
        return False

    aspect = aspect_x100(w, h)
    if aspect < MIN_ASPECT_X100 or aspect > MAX_ASPECT_X100:
        return False

    if density is not None:
        if density < MIN_DENSITY_PERCENT or density > MAX_DENSITY_PERCENT:
            return False

    return True


def rect_magnitude(rect):
    try:
        return rect.magnitude()
    except Exception:
        return 0


def get_rect_corners(rect, x, y, w, h):
    try:
        return rect.corners()
    except Exception:
        return bbox_corners(x, y, w, h)


def candidate_score(x, y, w, h, pixels, magnitude):
    center_x = x + w // 2
    center_y = y + h // 2
    center_penalty = (abs(center_x - IMG_CENTER_X) + abs(center_y - IMG_CENTER_Y)) // 4
    return (w * h) + (pixels * 5) + magnitude - center_penalty


def better_candidate(best, candidate):
    if best is None:
        return candidate
    if candidate[0] > best[0]:
        return candidate
    return best


def rect_candidate(rect):
    x, y, w, h = rect.rect()
    if not classify_bbox(x, y, w, h, None):
        return None

    corners = get_rect_corners(rect, x, y, w, h)
    cx, cy = average_center(corners)
    pixels = w * h
    score = candidate_score(x, y, w, h, pixels, rect_magnitude(rect))
    return (score, TARGET_METHOD_RECT, cx, cy, pixels, (x, y, w, h), corners)


def blob_candidate(blob):
    x = blob.x()
    y = blob.y()
    w = blob.w()
    h = blob.h()
    density = density_percent(blob.pixels(), w, h)
    if not classify_bbox(x, y, w, h, density):
        return None

    corners = bbox_corners(x, y, w, h)
    cx, cy = average_center(corners)
    score = candidate_score(x, y, w, h, blob.pixels(), 0)
    return (score, TARGET_METHOD_BLOB, cx, cy, blob.pixels(), (x, y, w, h), corners)


def group_candidate_from_blobs(blobs):
    min_x = IMG_W
    min_y = IMG_H
    max_x = 0
    max_y = 0
    pixels = 0
    parts = 0

    for blob in blobs:
        x = blob.x()
        y = blob.y()
        w = blob.w()
        h = blob.h()

        if is_border_bbox(x, y, w, h):
            continue
        if w > BLACK_GROUP_MAX_SINGLE_W or h > BLACK_GROUP_MAX_SINGLE_H:
            continue

        if x < min_x:
            min_x = x
        if y < min_y:
            min_y = y
        if x + w > max_x:
            max_x = x + w
        if y + h > max_y:
            max_y = y + h
        pixels += blob.pixels()
        parts += 1

    if parts < BLACK_GROUP_MIN_PARTS:
        return None

    w = max_x - min_x
    h = max_y - min_y
    density = density_percent(pixels, w, h)
    if not classify_bbox(min_x, min_y, w, h, density):
        return None

    corners = bbox_corners(min_x, min_y, w, h)
    cx, cy = average_center(corners)
    score = candidate_score(min_x, min_y, w, h, pixels, parts * 180)
    return (score, TARGET_METHOD_GROUP, cx, cy, pixels, (min_x, min_y, w, h), corners)


def find_black_frame_target(img):
    best = None

    try:
        rects = img.find_rects(threshold=RECT_THRESHOLD)
        for rect in rects:
            candidate = rect_candidate(rect)
            if candidate:
                best = better_candidate(best, candidate)
    except Exception:
        pass

    blobs = img.find_blobs(
        BLACK_THRESHOLDS,
        pixels_threshold=BLACK_MIN_PIXELS,
        area_threshold=BLACK_MIN_AREA,
        merge=True,
        margin=BLACK_MERGE_MARGIN,
    )
    for blob in blobs:
        candidate = blob_candidate(blob)
        if candidate:
            best = better_candidate(best, candidate)

    if best is None:
        raw_blobs = img.find_blobs(
            BLACK_THRESHOLDS,
            pixels_threshold=BLACK_MIN_PIXELS,
            area_threshold=BLACK_MIN_AREA,
            merge=False,
        )
        candidate = group_candidate_from_blobs(raw_blobs)
        if candidate:
            best = better_candidate(best, candidate)

    return best


def draw_target(img, target):
    score, method, cx, cy, pixels, rect, corners = target
    color = (0, 255, 0)
    if method == TARGET_METHOD_RECT:
        label = "RECT"
    elif method == TARGET_METHOD_GROUP:
        label = "GROUP"
    else:
        label = "BLOB"

    for index in range(len(corners)):
        x1, y1 = corners[index]
        x2, y2 = corners[(index + 1) % len(corners)]
        img.draw_line((x1, y1, x2, y2), color=color)
        img.draw_cross(x1, y1, color=color, size=5)

    img.draw_cross(cx, cy, color=(0, 255, 0), size=14)
    img.draw_string(
        rect[0],
        max(0, rect[1] - 10),
        "%s %d,%d" % (label, cx, cy),
        color=color,
    )


def safe_sensor_init():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_hmirror(CAMERA_HMIRROR)
    sensor.set_vflip(CAMERA_VFLIP)
    sensor.skip_frames(time=1200)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(True)


def capture_measure(enable_target_detection):
    global g_last_target, g_target_hold_frames

    clock.tick()
    img = sensor.snapshot()
    l_mean = scene_l_mean(img)
    fps_x10 = int(clock.fps() * 10)

    detected = False
    cx = -1
    cy = -1
    pixels = 0

    target = None
    if enable_target_detection:
        target = find_black_frame_target(img)
        if target:
            g_last_target = target
            g_target_hold_frames = TARGET_HOLD_FRAMES
        elif g_last_target is not None and g_target_hold_frames > 0:
            target = g_last_target
            g_target_hold_frames -= 1
        else:
            g_last_target = None
            g_target_hold_frames = 0

        if target:
            score, method, cx, cy, pixels, rect, corners = target
            detected = True
            draw_target(img, target)

    return detected, cx, cy, pixels, fps_x10, l_mean


def update_leds(target_detected):
    global g_last_led_ms
    now = millis()
    if now - g_last_led_ms >= 500:
        led_blue.toggle()
        g_last_led_ms = now

    if target_detected:
        led_green.on()
    else:
        led_green.off()


def write_frame(frame_type, detected, cx, cy, pixels, fps_x10, l_mean):
    flags = FLAG_CAMERA_OK
    if g_uart_rx_seen:
        flags |= FLAG_UART_RX_SEEN
    if detected:
        flags |= FLAG_TARGET_DETECTED
    uart.write(make_frame(frame_type, flags, cx, cy, pixels, fps_x10, l_mean))


def send_reply(cmd):
    global g_uart_rx_seen

    if cmd == CMD_PING:
        frame_type = TYPE_PING
        detect_color = False
    elif cmd == CMD_STATS:
        frame_type = TYPE_STATS
        detect_color = False
    elif cmd == CMD_COLOR:
        frame_type = TYPE_COLOR
        detect_color = True
    else:
        return

    g_uart_rx_seen = True
    detected, cx, cy, pixels, fps_x10, l_mean = capture_measure(detect_color)
    write_frame(frame_type, detected, cx, cy, pixels, fps_x10, l_mean)
    update_leds(detected)


def standalone_tick():
    global g_last_print_ms, g_last_stream_ms
    detected, cx, cy, pixels, fps_x10, l_mean = capture_measure(True)
    update_leds(detected)

    now = millis()
    if now - g_last_stream_ms >= UART_STREAM_INTERVAL_MS:
        write_frame(TYPE_COLOR, detected, cx, cy, pixels, fps_x10, l_mean)
        g_last_stream_ms = now

    if ENABLE_DEBUG_PRINT and now - g_last_print_ms >= 1000:
        print(
            "OMV black-frame fps=%.1f l=%d target=%d cx=%d cy=%d pixels=%d hold=%d uart_seen=%d exc=%d"
            % (
                fps_x10 / 10.0,
                l_mean,
                1 if detected else 0,
                cx,
                cy,
                pixels,
                g_target_hold_frames,
                1 if g_uart_rx_seen else 0,
                g_exception_count,
            )
        )
        g_last_print_ms = now


def drain_uart_input():
    global g_uart_rx_seen
    drained = 0
    while drained < UART_DRAIN_MAX_BYTES and uart.any():
        uart.readchar()
        g_uart_rx_seen = True
        drained += 1


def make_watchdog():
    if not ENABLE_WATCHDOG or WDT is None:
        return None
    try:
        return WDT(timeout=WATCHDOG_TIMEOUT_MS)
    except Exception:
        return None


def feed_watchdog():
    if g_wdt is not None:
        g_wdt.feed()


def recover_from_exception():
    led_red.on()
    led_green.off()
    led_blue.off()
    try:
        write_frame(TYPE_COLOR, False, -1, -1, 0, 0, 0)
    except Exception:
        pass
    time.sleep_ms(60)
    try:
        safe_sensor_init()
    except Exception:
        init_sensor_until_ok()
    led_red.off()


def init_sensor_until_ok():
    while True:
        try:
            safe_sensor_init()
            return
        except Exception:
            led_red.on()
            led_green.off()
            led_blue.off()
            time.sleep_ms(200)
            led_red.off()
            time.sleep_ms(200)


init_sensor_until_ok()
led_red.off()
led_green.off()
led_blue.off()
g_wdt = make_watchdog()

if ENABLE_DEBUG_PRINT:
    print("OpenMV black-frame UART stream ready: UART3=115200, QVGA RGB565, rot180=1")

while True:
    try:
        feed_watchdog()
        standalone_tick()
        drain_uart_input()
        g_loop_count += 1
        if (g_loop_count % GC_INTERVAL_FRAMES) == 0:
            gc.collect()
    except Exception:
        g_exception_count += 1
        recover_from_exception()
