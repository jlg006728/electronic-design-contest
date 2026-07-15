# OpenMV active UART stream for MSPM0 dual-servo laser aiming.
# 20260708 ring-center target version:
#   1. Keep the existing 17-byte UART protocol unchanged.
#   2. Prefer the smallest valid red ring near the bullseye.
#   3. Fall back to concentric-circle structure detection.
#   4. Keep red blob fallback disabled by default for laser safety.
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

ENABLE_DEBUG_PRINT = False
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

# Red/brown printed rings under indoor lighting. Tune these in OpenMV IDE if
# the marker is drawn with a different pen.
RED_THRESHOLDS = [
    (8, 95, 4, 90, -12, 90),
    (0, 100, 2, 80, -18, 95),
]

MIN_RING_PIXELS = 18
MIN_RING_AREA = 18
MIN_RING_W = 8
MIN_RING_H = 8
MIN_TARGET_BOX_W = 42
MIN_TARGET_BOX_H = 34
MAX_TARGET_ASPECT_X100 = 185
MIN_RING_DENSITY_PERCENT = 2
MAX_RING_DENSITY_PERCENT = 48

# Compact dense red objects are usually tape, LEDs, or reflections, not rings.
TAPE_DENSITY_PERCENT = 52
TAPE_MAX_W = 28
TAPE_MAX_H = 30
TAPE_MAX_AREA = 900

BORDER_REJECT_MARGIN = 3
RING_MERGE_MARGIN = 14
CENTER_PENALTY_DIVISOR = 5

# Primary circle detection. This rejects red glare because glare has no
# concentric-ring geometry.
ENABLE_SMALL_RED_RING_PRIMARY = True
ENABLE_CIRCLE_PRIMARY = True
ENABLE_RED_BLOB_FALLBACK = False
CIRCLE_SEARCH_ROI = (0, 0, IMG_W, IMG_H - 34)
CIRCLE_PRIMARY_THRESHOLD = 2200
CIRCLE_PRIMARY_MIN_RADIUS = 8
CIRCLE_PRIMARY_MAX_RADIUS = 92
CIRCLE_PRIMARY_R_STEP = 2
CIRCLE_GROUP_CENTER_MARGIN = 9
CIRCLE_GROUP_MIN_COUNT = 2
CIRCLE_GROUP_MAX_CENTER_Y = IMG_H - 42

# Optional Hough-circle refinement for the red-blob fallback.
ENABLE_CIRCLE_REFINE = False
CIRCLE_THRESHOLD = 2800
CIRCLE_ROI_PAD = 8

TARGET_METHOD_BLOB = 1
TARGET_METHOD_CIRCLE = 2
TARGET_METHOD_SMALL_RING = 3

SMALL_RING_MIN_W = 10
SMALL_RING_MIN_H = 10
SMALL_RING_MAX_W = 74
SMALL_RING_MAX_H = 74
SMALL_RING_MAX_ASPECT_X100 = 150
SMALL_RING_MIN_DENSITY_PERCENT = 4
SMALL_RING_MAX_DENSITY_PERCENT = 58
SMALL_RING_MIN_Y = 24
SMALL_RING_MAX_BOTTOM = IMG_H - 36
SMALL_RING_MERGE_MARGIN = 2


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


def aspect_x100(width, height):
    if width <= 0 or height <= 0:
        return 9999
    if width > height:
        return (width * 100) // height
    return (height * 100) // width


def is_near_border(blob):
    return (
        blob.x() <= BORDER_REJECT_MARGIN
        or blob.y() <= BORDER_REJECT_MARGIN
        or blob.x() + blob.w() >= IMG_W - BORDER_REJECT_MARGIN
        or blob.y() + blob.h() >= IMG_H - BORDER_REJECT_MARGIN
    )


def blob_density_percent(blob):
    try:
        return int(blob.density() * 100)
    except Exception:
        return 100


def is_probable_tape(blob, density_percent):
    compact = blob.w() <= TAPE_MAX_W or blob.h() <= TAPE_MAX_H
    return (
        density_percent >= TAPE_DENSITY_PERCENT
        and compact
        and blob.area() <= TAPE_MAX_AREA
    )


def ring_blob_score(blob):
    if is_near_border(blob):
        return None
    if blob.pixels() < MIN_RING_PIXELS or blob.area() < MIN_RING_AREA:
        return None
    if blob.w() < MIN_RING_W or blob.h() < MIN_RING_H:
        return None

    density_percent = blob_density_percent(blob)
    if density_percent < MIN_RING_DENSITY_PERCENT:
        return None
    if density_percent > MAX_RING_DENSITY_PERCENT:
        return None
    if is_probable_tape(blob, density_percent):
        return None
    if blob.w() < MIN_TARGET_BOX_W or blob.h() < MIN_TARGET_BOX_H:
        return None
    if aspect_x100(blob.w(), blob.h()) > MAX_TARGET_ASPECT_X100:
        return None

    box_cx = blob.x() + (blob.w() // 2)
    box_cy = blob.y() + (blob.h() // 2)
    dx = box_cx - IMG_CENTER_X
    dy = box_cy - IMG_CENTER_Y
    center_penalty = dx * dx + dy * dy
    square_penalty = abs(blob.w() - blob.h()) * 60

    return (
        blob.w() * blob.h() * 16
        + blob.pixels() * 900
        - square_penalty
        - (center_penalty // CENTER_PENALTY_DIVISOR)
    )


def clamp_roi(x, y, w, h):
    if x < 0:
        w += x
        x = 0
    if y < 0:
        h += y
        y = 0
    if x + w > IMG_W:
        w = IMG_W - x
    if y + h > IMG_H:
        h = IMG_H - y
    if w < 2:
        w = 2
    if h < 2:
        h = 2
    return (int(x), int(y), int(w), int(h))


def refine_center_with_circles(img, blob, cx, cy):
    if not ENABLE_CIRCLE_REFINE:
        return cx, cy, False

    roi = clamp_roi(
        blob.x() - CIRCLE_ROI_PAD,
        blob.y() - CIRCLE_ROI_PAD,
        blob.w() + (2 * CIRCLE_ROI_PAD),
        blob.h() + (2 * CIRCLE_ROI_PAD),
    )
    min_side = min(blob.w(), blob.h())
    max_side = max(blob.w(), blob.h())
    r_min = max(5, min_side // 8)
    r_max = min(115, max(10, max_side // 2))
    if r_max <= r_min:
        return cx, cy, False

    try:
        circles = img.find_circles(
            roi=roi,
            threshold=CIRCLE_THRESHOLD,
            x_margin=8,
            y_margin=8,
            r_margin=8,
            r_min=r_min,
            r_max=r_max,
            r_step=2,
        )
    except Exception:
        return cx, cy, False

    best = None
    best_score = -1000000000
    max_center_error = max(10, max_side // 4)
    for circle in circles:
        center_error = abs(circle.x() - cx) + abs(circle.y() - cy)
        if center_error > max_center_error:
            continue
        score = circle.magnitude() * 4 + circle.r() * 12 - center_error * 80
        if score > best_score:
            best_score = score
            best = circle

    if best is None:
        return cx, cy, False

    # Blend with the red-ring bounding box center to avoid Hough jitter.
    return ((2 * cx + best.x()) // 3), ((2 * cy + best.y()) // 3), True


def small_ring_blob_score(blob):
    if is_near_border(blob):
        return None
    if blob.y() < SMALL_RING_MIN_Y:
        return None
    if blob.y() + blob.h() > SMALL_RING_MAX_BOTTOM:
        return None
    if blob.w() < SMALL_RING_MIN_W or blob.h() < SMALL_RING_MIN_H:
        return None
    if blob.w() > SMALL_RING_MAX_W or blob.h() > SMALL_RING_MAX_H:
        return None
    if aspect_x100(blob.w(), blob.h()) > SMALL_RING_MAX_ASPECT_X100:
        return None

    density_percent = blob_density_percent(blob)
    if density_percent < SMALL_RING_MIN_DENSITY_PERCENT:
        return None
    if density_percent > SMALL_RING_MAX_DENSITY_PERCENT:
        return None
    if is_probable_tape(blob, density_percent):
        return None

    # Prefer the innermost valid ring, but keep a weak center preference so
    # small red noise near the image edge does not win.
    box_cx = blob.x() + (blob.w() // 2)
    box_cy = blob.y() + (blob.h() // 2)
    dx = box_cx - IMG_CENTER_X
    dy = box_cy - IMG_CENTER_Y
    center_penalty = dx * dx + dy * dy
    size_score = blob.w() * blob.h()
    circle_penalty = abs(blob.w() - blob.h()) * 120
    density_bonus = (SMALL_RING_MAX_DENSITY_PERCENT - density_percent) * 20

    return -size_score * 120 - circle_penalty + density_bonus - (
        center_penalty // CENTER_PENALTY_DIVISOR
    )


def find_small_red_ring_target(img):
    if not ENABLE_SMALL_RED_RING_PRIMARY:
        return None

    blobs = img.find_blobs(
        RED_THRESHOLDS,
        pixels_threshold=MIN_RING_PIXELS,
        area_threshold=MIN_RING_AREA,
        merge=True,
        margin=SMALL_RING_MERGE_MARGIN,
    )
    if not blobs:
        return None

    best = None
    best_score = -1000000000
    for blob in blobs:
        score = small_ring_blob_score(blob)
        if score is None:
            continue
        if score > best_score:
            best_score = score
            best = blob

    if best is None:
        return None

    cx = best.x() + (best.w() // 2)
    cy = best.y() + (best.h() // 2)
    radius = min(best.w(), best.h()) // 2
    return (TARGET_METHOD_SMALL_RING, cx, cy, best.pixels(), best.rect(), radius)


def find_circle_group_target(img):
    if not ENABLE_CIRCLE_PRIMARY:
        return None

    try:
        circles = img.find_circles(
            roi=CIRCLE_SEARCH_ROI,
            threshold=CIRCLE_PRIMARY_THRESHOLD,
            x_margin=8,
            y_margin=8,
            r_margin=8,
            r_min=CIRCLE_PRIMARY_MIN_RADIUS,
            r_max=CIRCLE_PRIMARY_MAX_RADIUS,
            r_step=CIRCLE_PRIMARY_R_STEP,
        )
    except Exception:
        return None

    if not circles:
        return None

    best_count = 0
    best_score = -1000000000
    best_cx = -1
    best_cy = -1
    best_r = 0

    for seed in circles:
        if seed.y() > CIRCLE_GROUP_MAX_CENTER_Y:
            continue

        count = 0
        weight_sum = 0
        x_sum = 0
        y_sum = 0
        max_radius = 0
        mag_sum = 0

        for circle in circles:
            if circle.y() > CIRCLE_GROUP_MAX_CENTER_Y:
                continue
            if abs(circle.x() - seed.x()) > CIRCLE_GROUP_CENTER_MARGIN:
                continue
            if abs(circle.y() - seed.y()) > CIRCLE_GROUP_CENTER_MARGIN:
                continue

            weight = max(1, int(circle.magnitude()))
            count += 1
            weight_sum += weight
            x_sum += circle.x() * weight
            y_sum += circle.y() * weight
            mag_sum += int(circle.magnitude())
            if circle.r() > max_radius:
                max_radius = circle.r()

        if count < CIRCLE_GROUP_MIN_COUNT or weight_sum <= 0:
            continue

        cx = x_sum // weight_sum
        cy = y_sum // weight_sum
        dx = cx - IMG_CENTER_X
        dy = cy - IMG_CENTER_Y
        center_penalty = dx * dx + dy * dy
        score = (
            count * 90000
            + mag_sum * 6
            + max_radius * 700
            - (center_penalty // CENTER_PENALTY_DIVISOR)
        )

        if score > best_score:
            best_score = score
            best_count = count
            best_cx = cx
            best_cy = cy
            best_r = max_radius

    if best_count == 0:
        return None

    pixels_estimate = max(100, best_count * 60 + best_r * 10)
    rect = clamp_roi(
        best_cx - best_r,
        best_cy - best_r,
        best_r * 2,
        best_r * 2,
    )
    return (TARGET_METHOD_CIRCLE, best_cx, best_cy, pixels_estimate, rect, best_r)


def find_ring_target(img):
    small_ring_target = find_small_red_ring_target(img)
    if small_ring_target:
        return small_ring_target

    circle_target = find_circle_group_target(img)
    if circle_target:
        return circle_target

    if not ENABLE_RED_BLOB_FALLBACK:
        return None

    blobs = img.find_blobs(
        RED_THRESHOLDS,
        pixels_threshold=MIN_RING_PIXELS,
        area_threshold=MIN_RING_AREA,
        merge=True,
        margin=RING_MERGE_MARGIN,
    )
    if not blobs:
        return None

    best = None
    best_score = -1000000000
    for blob in blobs:
        score = ring_blob_score(blob)
        if score is None:
            continue
        if score > best_score:
            best_score = score
            best = blob

    if best is None:
        return None

    cx = best.x() + (best.w() // 2)
    cy = best.y() + (best.h() // 2)
    cx, cy, circle_refined = refine_center_with_circles(img, best, cx, cy)
    radius = min(best.w(), best.h()) // 2 if circle_refined else 0
    return (TARGET_METHOD_BLOB, cx, cy, best.pixels(), best.rect(), radius)


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


def capture_measure(enable_color_detection):
    clock.tick()
    img = sensor.snapshot()
    l_mean = scene_l_mean(img)
    fps_x10 = int(clock.fps() * 10)

    detected = False
    cx = -1
    cy = -1
    pixels = 0

    if enable_color_detection:
        target = find_ring_target(img)
        if target:
            method, cx, cy, pixels, rect, radius = target
            detected = True
            img.draw_rectangle(rect, color=(0, 255, 0))
            if (
                (method == TARGET_METHOD_CIRCLE or method == TARGET_METHOD_SMALL_RING)
                and radius > 0
            ):
                img.draw_circle(cx, cy, radius, color=(255, 255, 0))
            img.draw_cross(cx, cy, color=(0, 255, 0), size=12)
            if method == TARGET_METHOD_BLOB and radius > 0:
                img.draw_cross(cx, cy, color=(255, 255, 0), size=8)

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
            "OMV ring fps=%.1f l=%d target=%d cx=%d cy=%d pixels=%d uart=%d exc=%d"
            % (
                fps_x10 / 10.0,
                l_mean,
                1 if detected else 0,
                cx,
                cy,
                pixels,
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
    safe_sensor_init()
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
    print("OpenMV UART ring-center stream ready: UART3=115200, QVGA RGB565")

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
