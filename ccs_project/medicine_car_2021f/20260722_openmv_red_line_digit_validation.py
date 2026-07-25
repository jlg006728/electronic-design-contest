"""2021 F题第一阶段 OpenMV 验证。

使用方法：在 OpenMV IDE 中修改 MODE 后运行：
  MODE = "red"            只验证红线检测；
  MODE = "digit_capture"  用 DIGIT_LABEL 学习一个数字模板；
  MODE = "digit_verify"   用已保存模板连续识别。

本脚本只做视觉可行性验证，不驱动电机，也不向整车发送控制量。
兼容 OpenMV 固件4.x（方法式 blob/rect API）和5.x（属性式 API）。
"""

import math
import sensor
import time
from pyb import LED, millis


IMG_W = 320
IMG_H = 240
CAMERA_HMIRROR = True
CAMERA_VFLIP = True

MODE = "red"
DIGIT_LABEL = "1"
TEMPLATE_FILE = "/flash/digit_templates.txt"

RED_THRESHOLDS = [
    (20, 80, 20, 90, 10, 80),
    (8, 98, 8, 110, -10, 100),
]
RED_BAND_HEIGHT = 18
RED_BAND_TOPS = (24, 58, 92, 126, 160, 194)
RED_MIN_PIXELS = 2
RED_MIN_AREA = 2
RED_MERGE_MARGIN = 8

BLACK_THRESHOLDS = [(0, 55, -35, 35, -35, 35)]
BOX_MIN_W = 22
BOX_MIN_H = 22
BOX_MAX_W = 230
BOX_MAX_H = 210
BOX_MIN_ASPECT_X100 = 45
BOX_MAX_ASPECT_X100 = 230
BOX_MIN_PIXELS = 8
BOX_MIN_AREA = 8
BOX_MERGE_MARGIN = 12
RECT_THRESHOLD = 7000

GRID_W = 16
GRID_H = 12
INNER_MARGIN_X100 = 18
INNER_MARGIN_Y100 = 18
DARK_LUMA_THRESHOLD = 95
CAPTURE_FRAMES = 12
CAPTURE_MAX_CENTER_JITTER = 8
VERIFY_HISTORY = 5
VERIFY_MIN_VOTES = 3
VERIFY_MIN_MARGIN = 1
VERIFY_MAX_DISTANCE = 55
PRINT_INTERVAL_MS = 250


clock = time.clock()
led_red = LED(1)
led_green = LED(2)
led_blue = LED(3)
last_print_ms = 0
last_blink_ms = 0


def crc8(data):
    crc = 0
    for value in data:
        crc ^= value
        for unused in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def packet(kind, fields):
    parts = [kind]
    for key, value in fields:
        parts.append("%s=%s" % (key, value))
    body = ",".join(parts)
    return "$%s,crc=%02X*\r\n" % (body, crc8(body.encode()))


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


def luma(pixel):
    if isinstance(pixel, tuple):
        return (pixel[0] * 30 + pixel[1] * 59 + pixel[2] * 11) // 100
    return pixel


def api_value(obj, name):
    """OpenMV 4.x exposes several blob/rect fields as methods, while 5.x
    exposes the same fields as integer properties.  Accept both forms."""

    value = getattr(obj, name)
    return value() if callable(value) else value


def red_band(img, top):
    blobs = img.find_blobs(
        RED_THRESHOLDS,
        roi=(0, top, IMG_W, RED_BAND_HEIGHT),
        pixels_threshold=RED_MIN_PIXELS,
        area_threshold=RED_MIN_AREA,
        merge=True,
        margin=RED_MERGE_MARGIN,
    )
    if not blobs:
        return None

    total_pixels = 0
    weighted_x = 0
    for blob in blobs:
        pixels = api_value(blob, "pixels")
        total_pixels += pixels
        weighted_x += api_value(blob, "cx") * pixels
    if total_pixels < RED_MIN_PIXELS:
        return None
    return (top + RED_BAND_HEIGHT // 2, weighted_x // total_pixels, total_pixels)


def estimate_red_line(bands):
    if len(bands) < 2:
        return None
    mean_y = sum(item[0] for item in bands) / len(bands)
    mean_x = sum(item[1] for item in bands) / len(bands)
    denom = sum((item[0] - mean_y) ** 2 for item in bands)
    if denom <= 0:
        return None
    slope = sum((item[0] - mean_y) * (item[1] - mean_x) for item in bands) / denom
    intercept = mean_x - slope * mean_y
    far_y = min(item[0] for item in bands)
    far_x = slope * far_y + intercept
    near_x = slope * (IMG_H - 1) + intercept
    center_x = (IMG_W - 1) / 2
    angle10 = int(math.degrees(math.atan2(near_x - far_x, (IMG_H - 1) - far_y)) * 10)
    return {
        "far_x": int(far_x),
        "near_x": int(near_x),
        "far_offset": int(far_x - center_x),
        "near_offset": int(near_x - center_x),
        "angle10": angle10,
        "slope1000": int(slope * 1000),
    }


def run_red(img):
    bands = []
    for top in RED_BAND_TOPS:
        result = red_band(img, top)
        if result is not None:
            bands.append(result)

    estimate = estimate_red_line(bands)
    if estimate is not None:
        img.draw_line(
            (estimate["far_x"], min(item[0] for item in bands), estimate["near_x"], IMG_H - 1),
            color=(0, 255, 0),
        )
        img.draw_cross((estimate["near_x"], IMG_H - 1), color=(0, 255, 0), size=10)
        img.draw_cross((estimate["far_x"], min(item[0] for item in bands)), color=(0, 255, 0), size=10)
        led_green.on()
        led_red.off()
        fields = [
            ("valid", 1),
            ("bands", len(bands)),
            ("near", estimate["near_x"]),
            ("far", estimate["far_x"]),
            ("near_off", estimate["near_offset"]),
            ("far_off", estimate["far_offset"]),
            ("angle10", estimate["angle10"]),
            ("pixels", sum(item[2] for item in bands)),
        ]
    else:
        led_green.off()
        if bands:
            led_red.on()
        else:
            led_red.off()
        fields = [("valid", 0), ("bands", len(bands)), ("near", -1), ("far", -1)]
    return fields


def box_aspect_ok(w, h):
    if h <= 0:
        return False
    aspect = (w * 100) // h
    return BOX_MIN_ASPECT_X100 <= aspect <= BOX_MAX_ASPECT_X100


def box_score(x, y, w, h, bonus):
    center_penalty = (abs(x + w // 2 - IMG_W // 2) + abs(y + h // 2 - IMG_H // 2)) // 3
    return w * h + bonus - center_penalty


def find_digit_box(img):
    candidates = []
    try:
        rects = img.find_rects(threshold=RECT_THRESHOLD)
    except Exception:
        rects = []
    for rect in rects:
        x = api_value(rect, "x")
        y = api_value(rect, "y")
        w = api_value(rect, "w")
        h = api_value(rect, "h")
        if BOX_MIN_W <= w <= BOX_MAX_W and BOX_MIN_H <= h <= BOX_MAX_H and box_aspect_ok(w, h):
            candidates.append((box_score(x, y, w, h, 15000), (x, y, w, h), "rect"))

    if not candidates:
        blobs = img.find_blobs(
            BLACK_THRESHOLDS,
            pixels_threshold=BOX_MIN_PIXELS,
            area_threshold=BOX_MIN_AREA,
            merge=True,
            margin=BOX_MERGE_MARGIN,
        )
        for blob in blobs:
            x = api_value(blob, "x")
            y = api_value(blob, "y")
            w = api_value(blob, "w")
            h = api_value(blob, "h")
            if BOX_MIN_W <= w <= BOX_MAX_W and BOX_MIN_H <= h <= BOX_MAX_H and box_aspect_ok(w, h):
                candidates.append((box_score(x, y, w, h, api_value(blob, "pixels") * 4), (x, y, w, h), "blob"))

    if not candidates:
        return None
    candidates.sort(key=lambda item: item[0], reverse=True)
    return candidates[0][1] + (candidates[0][2],)


def sample_signature(img, box):
    x, y, w, h = box[0], box[1], box[2], box[3]
    margin_x = (w * INNER_MARGIN_X100) // 100
    margin_y = (h * INNER_MARGIN_Y100) // 100
    left = x + margin_x
    top = y + margin_y
    inner_w = max(1, w - 2 * margin_x)
    inner_h = max(1, h - 2 * margin_y)
    bits = []
    for row in range(GRID_H):
        for col in range(GRID_W):
            cx = left + (col * 2 + 1) * inner_w // (GRID_W * 2)
            cy = top + (row * 2 + 1) * inner_h // (GRID_H * 2)
            dark = 0
            total = 0
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    px = min(IMG_W - 1, max(0, cx + dx))
                    py = min(IMG_H - 1, max(0, cy + dy))
                    if luma(img.get_pixel((px, py), rgbtuple=True)) <= DARK_LUMA_THRESHOLD:
                        dark += 1
                    total += 1
            bits.append("1" if dark * 2 >= total else "0")
    return "".join(bits)


def save_template(label, signature):
    templates = {}
    try:
        handle = open(TEMPLATE_FILE, "r")
        for line in handle:
            line = line.strip()
            if "|" in line:
                old_label, old_signature = line.split("|", 1)
                templates[old_label] = old_signature
        handle.close()
    except OSError:
        pass
    templates[label] = signature
    handle = open(TEMPLATE_FILE, "w")
    for old_label in sorted(templates):
        handle.write("%s|%s\n" % (old_label, templates[old_label]))
    handle.close()


def load_templates():
    templates = {}
    try:
        handle = open(TEMPLATE_FILE, "r")
        for line in handle:
            line = line.strip()
            if "|" in line:
                label, signature = line.split("|", 1)
                if len(signature) == GRID_W * GRID_H:
                    templates[label] = signature
        handle.close()
    except OSError:
        pass
    return templates


def hamming(left, right):
    if len(left) != len(right):
        return 9999
    distance = 0
    for index in range(len(left)):
        if left[index] != right[index]:
            distance += 1
    return distance


def rank_signature(signature, templates):
    ranked = []
    for label in templates:
        ranked.append((label, hamming(signature, templates[label])))
    ranked.sort(key=lambda item: item[1])
    return ranked


def capture_digit():
    global last_print_ms
    signatures = []
    last_center = None
    while True:
        clock.tick()
        img = sensor.snapshot()
        box = find_digit_box(img)
        if box is not None:
            x, y, w, h = box[0], box[1], box[2], box[3]
            center = (x + w // 2, y + h // 2)
            if last_center is None or (abs(center[0] - last_center[0]) <= CAPTURE_MAX_CENTER_JITTER and abs(center[1] - last_center[1]) <= CAPTURE_MAX_CENTER_JITTER):
                signatures.append(sample_signature(img, box))
            else:
                signatures = []
            last_center = center
            img.draw_rectangle((x, y, w, h), color=(0, 255, 0))
            led_green.on()
        else:
            signatures = []
            last_center = None
            led_green.off()
        if len(signatures) >= CAPTURE_FRAMES:
            bits = []
            for index in range(GRID_W * GRID_H):
                ones = sum(signature[index] == "1" for signature in signatures)
                bits.append("1" if ones * 2 >= len(signatures) else "0")
            signature = "".join(bits)
            save_template(DIGIT_LABEL, signature)
            print(packet("DIGIT_CAPTURE", [("label", DIGIT_LABEL), ("frames", len(signatures)), ("bits", len(signature))]))
            led_blue.on()
            return
        now = millis()
        if now - last_print_ms >= PRINT_INTERVAL_MS:
            print(packet("DIGIT_CAPTURE", [("label", DIGIT_LABEL), ("stable", len(signatures)), ("need", CAPTURE_FRAMES)]))
            last_print_ms = now


def verify_digits():
    global last_print_ms
    templates = load_templates()
    history = []
    while True:
        clock.tick()
        img = sensor.snapshot()
        box = find_digit_box(img)
        label = None
        distance = -1
        margin = 0
        if box is not None and templates:
            signature = sample_signature(img, box)
            ranked = rank_signature(signature, templates)
            if ranked:
                label, distance = ranked[0]
                second = ranked[1][1] if len(ranked) > 1 else distance + VERIFY_MIN_MARGIN
                margin = second - distance
                if distance > VERIFY_MAX_DISTANCE or margin < VERIFY_MIN_MARGIN:
                    label = None
            if label is not None:
                history.append(label)
                if len(history) > VERIFY_HISTORY:
                    history.pop(0)
            else:
                # Do not let a previous card vote through a lost/changed box.
                history = []
        elif box is None:
            history = []
        if box is not None:
            img.draw_rectangle((box[0], box[1], box[2], box[3]), color=(0, 255, 0) if label else (255, 180, 0))
        counts = {}
        for item in history:
            counts[item] = counts.get(item, 0) + 1
        voted = None
        votes = 0
        for item in counts:
            if counts[item] > votes:
                voted = item
                votes = counts[item]
        if votes < VERIFY_MIN_VOTES:
            voted = None
        now = millis()
        if now - last_print_ms >= PRINT_INTERVAL_MS:
            print(packet("DIGIT", [("templates", len(templates)), ("label", label or "none"), ("vote", voted or "none"), ("votes", votes), ("dist", distance), ("margin", margin), ("fps10", int(clock.fps() * 10))]))
            last_print_ms = now
        if voted:
            led_green.on()
        else:
            led_green.off()
        if box is not None and label is None:
            led_red.on()
        else:
            led_red.off()


def main():
    global last_print_ms, last_blink_ms
    sensor_init()
    led_red.off()
    led_green.off()
    led_blue.off()
    print("VISIONDBG mode=%s boot; motors=off" % MODE)
    if MODE == "digit_capture":
        capture_digit()
        return
    if MODE == "digit_verify":
        verify_digits()
        return
    while True:
        clock.tick()
        img = sensor.snapshot()
        fields = run_red(img)
        now = millis()
        if now - last_print_ms >= PRINT_INTERVAL_MS:
            fields.append(("fps10", int(clock.fps() * 10)))
            print(packet("RED", fields))
            last_print_ms = now
        if now - last_blink_ms >= 500:
            led_blue.toggle()
            last_blink_ms = now


main()
