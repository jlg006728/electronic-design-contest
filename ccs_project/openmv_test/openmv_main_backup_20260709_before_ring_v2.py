# Simple OpenMV red color sensitivity test.
#
# Run this script in OpenMV IDE. Red regions are boxed on the image window.
# Tune RED_THRESHOLDS, MIN_RED_PIXELS, and MIN_RED_AREA while watching whether
# the target is detected and whether background objects are falsely boxed.

import sensor
import time
from pyb import LED, millis


IMG_W = 320
IMG_H = 240

# Keep these consistent with the current mechanical installation.
CAMERA_HMIRROR = True
CAMERA_VFLIP = True

# LAB thresholds for red in OpenMV format: (L min, L max, A min, A max, B min, B max).
# Range 1 is relatively conservative; range 2 is more sensitive for darker/farther red.
RED_THRESHOLDS = [
    (20, 80, 20, 90, 10, 80),
    (10, 95, 12, 110, -5, 95),
]

MIN_RED_PIXELS = 20
MIN_RED_AREA = 20
MERGE_MARGIN = 8
PRINT_INTERVAL_MS = 300


clock = time.clock()
led_green = LED(2)
led_blue = LED(3)
last_print_ms = 0
last_blink_ms = 0


def sensor_init():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_hmirror(CAMERA_HMIRROR)
    sensor.set_vflip(CAMERA_VFLIP)
    sensor.skip_frames(time=1200)

    # Lock gain and white balance so red threshold behavior is easier to judge.
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(True)


def find_red_blobs(img):
    return img.find_blobs(
        RED_THRESHOLDS,
        pixels_threshold=MIN_RED_PIXELS,
        area_threshold=MIN_RED_AREA,
        merge=True,
        margin=MERGE_MARGIN,
    )


def draw_blob(img, blob, is_best):
    color = (0, 255, 0) if is_best else (255, 180, 0)
    img.draw_rectangle(blob.rect(), color=color)
    img.draw_cross(blob.cx(), blob.cy(), color=color, size=10 if is_best else 6)

    if is_best:
        img.draw_string(
            blob.x(),
            max(0, blob.y() - 10),
            "red %d" % blob.pixels(),
            color=color,
        )


def choose_best_blob(blobs):
    best = None
    for blob in blobs:
        if best is None or blob.pixels() > best.pixels():
            best = blob
    return best


sensor_init()
led_green.off()
led_blue.off()

print("Simple red color sensitivity test ready.")
print("Tune RED_THRESHOLDS / MIN_RED_PIXELS / MIN_RED_AREA in OpenMV IDE.")

while True:
    clock.tick()
    img = sensor.snapshot()
    blobs = find_red_blobs(img)

    best = None
    if blobs:
        best = choose_best_blob(blobs)
        for blob in blobs:
            draw_blob(img, blob, blob == best)
        led_green.on()
    else:
        led_green.off()

    now = millis()
    if now - last_blink_ms >= 500:
        led_blue.toggle()
        last_blink_ms = now

    if now - last_print_ms >= PRINT_INTERVAL_MS:
        if best:
            print(
                "red=1 blobs=%d cx=%d cy=%d pixels=%d rect=%s fps=%.1f"
                % (
                    len(blobs),
                    best.cx(),
                    best.cy(),
                    best.pixels(),
                    best.rect(),
                    clock.fps(),
                )
            )
        else:
            print("red=0 blobs=0 fps=%.1f" % clock.fps())
        last_print_ms = now
