# OpenMV H7 script for 2025 E target aiming.
# UART frame to MSPM0:
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

# Tune these in OpenMV IDE: Tools -> Machine Vision -> Threshold Editor.
# Red marker / red circle under indoor light, LAB color space.
RED_THRESHOLDS = [
    (20, 80, 20, 90, 10, 80),
]

MIN_RED_PIXELS = 8
MIN_RED_AREA = 8


uart = UART(3, UART_BAUD, timeout_char=20)


def checksum(payload):
    x = 0
    for b in payload:
        x ^= b
    return x


def send_target(detected, cx, cy):
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

    # The target center mark is small and close to the image center after rough
    # alignment, so score by both red area and center distance.
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

clock = time.clock()

while True:
    clock.tick()
    if uart.any():
        cmd = uart.readchar()
        if cmd == REQUEST_BYTE:
            snapshot_and_send()
    else:
        # Keep the sensor exposure fresh while waiting for MSPM0 requests.
        sensor.snapshot()
