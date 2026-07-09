# OpenMV red ring sensitivity test.
#
# Run this script in OpenMV IDE. It detects thin red ring-like targets, boxes
# candidates, and prints obvious RINGDBG feedback in the serial terminal.

import sensor
import time
from pyb import LED, millis


IMG_W = 320
IMG_H = 240

# Keep these consistent with the current mechanical installation.
CAMERA_HMIRROR = True
CAMERA_VFLIP = True

# LAB thresholds for red in OpenMV format: (L min, L max, A min, A max, B min, B max).
# Range 1 is normal red; range 2 catches weak/pale red ring pixels.
RED_THRESHOLDS = [
    (20, 80, 20, 90, 10, 80),
    (8, 98, 8, 110, -10, 100),
]

# These are intentionally low because the target is a thin line, not a solid blob.
MIN_RED_PIXELS = 4
MIN_RED_AREA = 4
MERGE_MARGIN = 18

# Ring candidate gates. Keep broad first, then tighten from terminal feedback.
BORDER_MARGIN = 5
MIN_RING_W = 8
MIN_RING_H = 8
MAX_RING_W = 180
MAX_RING_H = 160
MIN_ASPECT_X100 = 45
MAX_ASPECT_X100 = 220
MIN_DENSITY_PERCENT = 2
MAX_DENSITY_PERCENT = 70

DRAW_REJECTED_BLOBS = True
PRINT_INTERVAL_MS = 250


clock = time.clock()
led_red = LED(1)
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


def find_raw_red_blobs(img):
    return img.find_blobs(
        RED_THRESHOLDS,
        pixels_threshold=MIN_RED_PIXELS,
        area_threshold=MIN_RED_AREA,
        merge=True,
        margin=MERGE_MARGIN,
    )


def is_border_blob(blob):
    if blob.x() <= BORDER_MARGIN:
        return True
    if blob.y() <= BORDER_MARGIN:
        return True
    if blob.x() + blob.w() >= IMG_W - BORDER_MARGIN:
        return True
    if blob.y() + blob.h() >= IMG_H - BORDER_MARGIN:
        return True
    return False


def blob_density_percent(blob):
    area = blob.w() * blob.h()
    if area <= 0:
        return 0
    return (blob.pixels() * 100) // area


def aspect_x100(blob):
    if blob.h() <= 0:
        return 999
    return (blob.w() * 100) // blob.h()


def classify_ring_candidate(blob):
    if is_border_blob(blob):
        return "edge"

    if (
        blob.w() < MIN_RING_W
        or blob.h() < MIN_RING_H
        or blob.w() > MAX_RING_W
        or blob.h() > MAX_RING_H
    ):
        return "size"

    aspect = aspect_x100(blob)
    if aspect < MIN_ASPECT_X100 or aspect > MAX_ASPECT_X100:
        return "ratio"

    density = blob_density_percent(blob)
    if density < MIN_DENSITY_PERCENT or density > MAX_DENSITY_PERCENT:
        return "density"

    return "ok"


def candidate_score(blob):
    density = blob_density_percent(blob)
    center_dx = abs(blob.cx() - IMG_W // 2)
    center_dy = abs(blob.cy() - IMG_H // 2)
    size_score = blob.w() * blob.h()
    pixel_score = blob.pixels() * 6
    density_bonus = 60 - abs(density - 18)
    center_penalty = (center_dx + center_dy) // 3
    return size_score + pixel_score + density_bonus - center_penalty


def draw_blob(img, blob, state, is_best):
    if is_best:
        color = (0, 255, 0)
    elif state == "ok":
        color = (255, 180, 0)
    else:
        color = (255, 0, 0)

    img.draw_rectangle(blob.rect(), color=color)
    img.draw_cross(blob.cx(), blob.cy(), color=color, size=12 if is_best else 6)

    if is_best:
        img.draw_string(
            blob.x(),
            max(0, blob.y() - 10),
            "RING p=%d d=%d%%" % (blob.pixels(), blob_density_percent(blob)),
            color=color,
        )


def analyze_blobs(blobs):
    best = None
    best_score = -1000000
    accepted = []
    reject_edge = 0
    reject_size = 0
    reject_ratio = 0
    reject_density = 0

    for blob in blobs:
        state = classify_ring_candidate(blob)
        if state == "ok":
            accepted.append(blob)
            score = candidate_score(blob)
            if best is None or score > best_score:
                best_score = score
                best = blob
        elif state == "edge":
            reject_edge += 1
        elif state == "size":
            reject_size += 1
        elif state == "ratio":
            reject_ratio += 1
        else:
            reject_density += 1

    return best, accepted, reject_edge, reject_size, reject_ratio, reject_density


def format_best(best):
    if best is None:
        return "best=none"
    return (
        "best=1 cx=%d cy=%d p=%d d=%d%% rect=%s"
        % (
            best.cx(),
            best.cy(),
            best.pixels(),
            blob_density_percent(best),
            best.rect(),
        )
    )


def update_leds(raw_count, best):
    if best is not None:
        led_red.off()
        led_green.on()
    elif raw_count > 0:
        led_red.on()
        led_green.off()
    else:
        led_red.off()
        led_green.off()


def draw_debug_blobs(img, blobs, best):
    for blob in blobs:
        state = classify_ring_candidate(blob)
        if state != "ok" and not DRAW_REJECTED_BLOBS:
            continue
        draw_blob(img, blob, state, blob == best)


sensor_init()
led_red.off()
led_green.off()
led_blue.off()

print("RINGDBG boot red ring sensitivity test")
print("RINGDBG colors: green=best ring, orange=accepted, red=rejected")
print("RINGDBG leds: green=ring ok, red=red seen but rejected")

while True:
    clock.tick()
    img = sensor.snapshot()
    blobs = find_raw_red_blobs(img)
    best, accepted, edge, size, ratio, density = analyze_blobs(blobs)

    draw_debug_blobs(img, blobs, best)
    update_leds(len(blobs), best)

    now = millis()
    if now - last_blink_ms >= 500:
        led_blue.toggle()
        last_blink_ms = now

    if now - last_print_ms >= PRINT_INTERVAL_MS:
        print(
            "RINGDBG raw=%d ok=%d edge=%d size=%d ratio=%d dens=%d %s fps=%.1f"
            % (
                len(blobs),
                len(accepted),
                edge,
                size,
                ratio,
                density,
                format_best(best),
                clock.fps(),
            )
        )
        last_print_ms = now
