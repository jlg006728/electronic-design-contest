# OpenMV H7 UART camera test for MSPM0G3507.
#
# Wiring for OpenMV H7 UART(3):
#   OpenMV P4 / UART3_TX -> MSPM0 PA11 / UART0_RX
#   OpenMV P5 / UART3_RX <- MSPM0 PA10 / UART0_TX
#   OpenMV GND           -> MSPM0 GND
#
# MSPM0 sends request byte 'X'. OpenMV replies:
#   [0x2C, detected, cx_l, cx_h, cy_l, cy_h, xor, 0x5B]

import sensor
import time
from pyb import UART


UART_BAUD = 115200
FRAME_HEADER = 0x2C
FRAME_FOOTER = 0x5B
REQUEST_BYTE = ord("X")

IMG_W = 320
IMG_H = 240
IMG_CENTER_X = IMG_W // 2
IMG_CENTER_Y = IMG_H // 2

# Adjust in OpenMV IDE if needed: Tools -> Machine Vision -> Threshold Editor.
# Default target: red marker / red dot under indoor light.
RED_THRESHOLDS = [
    (20, 80, 20, 90, 10, 80),
]

MIN_RED_PIXELS = 8
MIN_RED_AREA = 8

uart = UART(3, UART_BAUD, timeout_char=20)
clock = time.clock()
reply_count = 0


def checksum(payload):
    value = 0
    for b in payload:
        value ^= b
    return value


def send_target(detected, cx, cy):
    global reply_count
    if not detected:
        cx = 0xFFFF
        cy = 0xFFFF

    payload = bytearray([
        1 if detected else 0,
        cx & 0xFF,
        (cx >> 8) & 0xFF,
        cy & 0xFF,
        (cy >> 8) & 0xFF,
    ])

    frame = bytearray([FRAME_HEADER]) + payload
    frame.append(checksum(payload))
    frame.append(FRAME_FOOTER)
    uart.write(frame)
    reply_count += 1


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
    best_score = -1000000
    for blob in blobs:
        dx = blob.cx() - IMG_CENTER_X
        dy = blob.cy() - IMG_CENTER_Y
        center_penalty = dx * dx + dy * dy
        score = blob.pixels() * 100 - center_penalty
        if score > best_score:
            best_score = score
            best = blob
    return best


def snapshot_and_send():
    img = sensor.snapshot()
    blob = find_red_target(img)
    if blob:
        img.draw_rectangle(blob.rect(), color=(0, 255, 0))
        img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))
        send_target(True, blob.cx(), blob.cy())
    else:
        send_target(False, 0, 0)


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=1200)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(True)

while True:
    clock.tick()

    if uart.any():
        cmd = uart.readchar()
        if cmd == REQUEST_BYTE:
            snapshot_and_send()
            if reply_count % 20 == 0:
                print("reply:", reply_count, "fps:", clock.fps())
    else:
        # Keep exposure fresh while waiting for MSPM0 requests.
        sensor.snapshot()
