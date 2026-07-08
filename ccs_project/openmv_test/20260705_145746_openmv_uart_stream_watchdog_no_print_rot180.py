# OpenMV active UART stream for MSPM0 dual-servo tracking.
# 20260705_145746:
#   1. Keep streaming color frames without waiting for MSPM0 commands.
#   2. Never print during standalone operation.
#   3. Drain UART RX with a hard byte limit so RX noise cannot starve vision.
#   4. Catch Python-level camera/blob/UART exceptions and reinitialize camera.
#   5. Enable watchdog when available so a firmware-level hang can auto-reset.
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

# Red target default threshold. Tune in OpenMV IDE threshold editor if lighting
# changes a lot.
RED_THRESHOLDS = [
    (20, 80, 20, 90, 10, 80),
]

MIN_RED_PIXELS = 25
MIN_RED_AREA = 25


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


def find_red_target(img):
    blobs = img.find_blobs(
        RED_THRESHOLDS,
        pixels_threshold=MIN_RED_PIXELS,
        area_threshold=MIN_RED_AREA,
        merge=True,
        margin=3,
    )
    if not blobs:
        return None

    best = None
    best_score = -1000000000
    for blob in blobs:
        dx = blob.cx() - IMG_CENTER_X
        dy = blob.cy() - IMG_CENTER_Y
        center_penalty = dx * dx + dy * dy
        score = blob.pixels() * 1000 - center_penalty
        if score > best_score:
            best_score = score
            best = blob
    return best


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
        blob = find_red_target(img)
        if blob:
            detected = True
            cx = blob.cx()
            cy = blob.cy()
            pixels = blob.pixels()
            img.draw_rectangle(blob.rect(), color=(0, 255, 0))
            img.draw_cross(cx, cy, color=(0, 255, 0))

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
            "OMV stream fps=%.1f l=%d target=%d cx=%d cy=%d pixels=%d uart_seen=%d exc=%d"
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
    print("OpenMV UART watchdog stream ready: UART3=115200, QVGA RGB565, rot180=1")

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
