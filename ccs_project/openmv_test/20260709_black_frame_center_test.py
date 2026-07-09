# OpenMV black frame center test.
#
# Detect the black frame around the target, estimate four corner points, and
# report the averaged center as cx/cy. Terminal output uses BOXDBG lines.

import sensor
import time
from pyb import LED, millis


IMG_W = 320
IMG_H = 240

CAMERA_HMIRROR = True
CAMERA_VFLIP = True

# RGB565 LAB threshold for black/dark tape: (L min, L max, A min, A max, B min, B max).
BLACK_THRESHOLDS = [
    (0, 45, -30, 30, -30, 30),
]

RECT_THRESHOLD = 9000
BLACK_MIN_PIXELS = 8
BLACK_MIN_AREA = 8
BLACK_MERGE_MARGIN = 20

BORDER_MARGIN = 6
MIN_BOX_W = 14
MIN_BOX_H = 14
MAX_BOX_W = 170
MAX_BOX_H = 160
MIN_ASPECT_X100 = 45
MAX_ASPECT_X100 = 230
MIN_DENSITY_PERCENT = 3
MAX_DENSITY_PERCENT = 85

DRAW_REJECTED = True
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

    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(True)


def bbox_from_rect(rect):
    x, y, w, h = rect.rect()
    return x, y, w, h


def bbox_from_blob(blob):
    return blob.x(), blob.y(), blob.w(), blob.h()


def bbox_corners(x, y, w, h):
    return [
        (x, y),
        (x + w, y),
        (x + w, y + h),
        (x, y + h),
    ]


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
    if x <= BORDER_MARGIN:
        return True
    if y <= BORDER_MARGIN:
        return True
    if x + w >= IMG_W - BORDER_MARGIN:
        return True
    if y + h >= IMG_H - BORDER_MARGIN:
        return True
    return False


def classify_bbox(x, y, w, h, density):
    if is_border_bbox(x, y, w, h):
        return "edge"

    if w < MIN_BOX_W or h < MIN_BOX_H or w > MAX_BOX_W or h > MAX_BOX_H:
        return "size"

    aspect = aspect_x100(w, h)
    if aspect < MIN_ASPECT_X100 or aspect > MAX_ASPECT_X100:
        return "ratio"

    if density is not None:
        if density < MIN_DENSITY_PERCENT or density > MAX_DENSITY_PERCENT:
            return "density"

    return "ok"


def rect_magnitude(rect):
    try:
        return rect.magnitude()
    except Exception:
        return 0


def candidate_score(x, y, w, h, pixels, magnitude):
    center_x = x + w // 2
    center_y = y + h // 2
    center_penalty = (abs(center_x - IMG_W // 2) + abs(center_y - IMG_H // 2)) // 4
    return (w * h) + (pixels * 5) + magnitude - center_penalty


def add_reject(rejects, state):
    if state == "edge":
        rejects[0] += 1
    elif state == "size":
        rejects[1] += 1
    elif state == "ratio":
        rejects[2] += 1
    else:
        rejects[3] += 1


def make_candidate(kind, x, y, w, h, corners, pixels, density, magnitude):
    center_x, center_y = average_center(corners)
    return {
        "kind": kind,
        "x": x,
        "y": y,
        "w": w,
        "h": h,
        "corners": corners,
        "cx": center_x,
        "cy": center_y,
        "pixels": pixels,
        "density": density,
        "score": candidate_score(x, y, w, h, pixels, magnitude),
    }


def get_rect_corners(rect, x, y, w, h):
    try:
        return rect.corners()
    except Exception:
        return bbox_corners(x, y, w, h)


def find_rect_candidates(img, rejects):
    candidates = []
    try:
        rects = img.find_rects(threshold=RECT_THRESHOLD)
    except Exception:
        return [], 0

    for rect in rects:
        x, y, w, h = bbox_from_rect(rect)
        state = classify_bbox(x, y, w, h, None)
        if state != "ok":
            add_reject(rejects, state)
            if DRAW_REJECTED:
                img.draw_rectangle((x, y, w, h), color=(255, 0, 0))
            continue

        corners = get_rect_corners(rect, x, y, w, h)
        candidates.append(
            make_candidate(
                "rect",
                x,
                y,
                w,
                h,
                corners,
                w * h,
                None,
                rect_magnitude(rect),
            )
        )

    return candidates, len(rects)


def find_blob_candidates(img, rejects):
    blobs = img.find_blobs(
        BLACK_THRESHOLDS,
        pixels_threshold=BLACK_MIN_PIXELS,
        area_threshold=BLACK_MIN_AREA,
        merge=True,
        margin=BLACK_MERGE_MARGIN,
    )
    candidates = []

    for blob in blobs:
        x, y, w, h = bbox_from_blob(blob)
        density = density_percent(blob.pixels(), w, h)
        state = classify_bbox(x, y, w, h, density)
        if state != "ok":
            add_reject(rejects, state)
            if DRAW_REJECTED:
                img.draw_rectangle((x, y, w, h), color=(255, 0, 0))
            continue

        candidates.append(
            make_candidate(
                "blob",
                x,
                y,
                w,
                h,
                bbox_corners(x, y, w, h),
                blob.pixels(),
                density,
                0,
            )
        )

    return candidates, len(blobs)


def choose_best(candidates):
    best = None
    for candidate in candidates:
        if best is None or candidate["score"] > best["score"]:
            best = candidate
    return best


def draw_candidate(img, candidate, is_best):
    color = (0, 255, 0) if is_best else (255, 180, 0)
    corners = candidate["corners"]
    count = len(corners)

    for index in range(count):
        x1, y1 = corners[index]
        x2, y2 = corners[(index + 1) % count]
        img.draw_line((x1, y1, x2, y2), color=color)
        img.draw_cross(x1, y1, color=color, size=5)

    img.draw_cross(candidate["cx"], candidate["cy"], color=color, size=14 if is_best else 8)

    if is_best:
        img.draw_string(
            candidate["x"],
            max(0, candidate["y"] - 10),
            "BOX %s %d,%d" % (candidate["kind"], candidate["cx"], candidate["cy"]),
            color=color,
        )


def corners_text(candidate):
    if candidate is None:
        return "corners=none"

    text = "corners="
    for x, y in candidate["corners"]:
        text += "(%d,%d)" % (x, y)
    return text


def best_text(candidate):
    if candidate is None:
        return "best=none"

    density = candidate["density"]
    if density is None:
        density = -1

    return (
        "best=%s cx=%d cy=%d box=(%d,%d,%d,%d) p=%d d=%d%% %s"
        % (
            candidate["kind"],
            candidate["cx"],
            candidate["cy"],
            candidate["x"],
            candidate["y"],
            candidate["w"],
            candidate["h"],
            candidate["pixels"],
            density,
            corners_text(candidate),
        )
    )


def update_leds(best, raw_rects, raw_blobs):
    if best is not None:
        led_red.off()
        led_green.on()
    elif raw_rects > 0 or raw_blobs > 0:
        led_red.on()
        led_green.off()
    else:
        led_red.off()
        led_green.off()


sensor_init()
led_red.off()
led_green.off()
led_blue.off()

print("BOXDBG boot black frame center test")
print("BOXDBG colors: green=best, orange=accepted, red=rejected")
print("BOXDBG method: find_rects first, black blob fallback")

while True:
    clock.tick()
    img = sensor.snapshot()

    rejects = [0, 0, 0, 0]
    rect_candidates, raw_rects = find_rect_candidates(img, rejects)
    blob_candidates, raw_blobs = find_blob_candidates(img, rejects)
    candidates = rect_candidates + blob_candidates
    best = choose_best(candidates)

    for candidate in candidates:
        draw_candidate(img, candidate, candidate == best)

    update_leds(best, raw_rects, raw_blobs)

    now = millis()
    if now - last_blink_ms >= 500:
        led_blue.toggle()
        last_blink_ms = now

    if now - last_print_ms >= PRINT_INTERVAL_MS:
        print(
            "BOXDBG rects=%d blobs=%d ok=%d edge=%d size=%d ratio=%d dens=%d %s fps=%.1f"
            % (
                raw_rects,
                raw_blobs,
                len(candidates),
                rejects[0],
                rejects[1],
                rejects[2],
                rejects[3],
                best_text(best),
                clock.fps(),
            )
        )
        last_print_ms = now
