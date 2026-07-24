"""数字识别的纯逻辑层，不依赖 CanMV、Sensor 或 YOLO 硬件模块。"""


IMAGE_WIDTH = 640
IMAGE_HEIGHT = 360
CENTER_DEADBAND_X = 32

SIDE_CENTER = 0
SIDE_LEFT = 1
SIDE_RIGHT = 2

DIGIT_FLAG_VALID = 0x01
DIGIT_FLAG_TARGET_MATCH = 0x02
DIGIT_FLAG_CONSENSUS = 0x04
DIGIT_FLAG_LOCKED = 0x08
DIGIT_FLAG_ERROR = 0x10

VISUAL_MODE_OFF = 0
VISUAL_MODE_PHARMACY = 1
VISUAL_MODE_TARGET = 2


class DigitDetection:
    """规范化后的单个数字检测结果。"""

    __slots__ = (
        "digit",
        "x",
        "y",
        "width",
        "height",
        "side",
        "confidence",
    )

    def __init__(
        self,
        digit,
        x,
        y,
        width,
        height,
        side,
        confidence,
    ):
        self.digit = int(digit)
        self.x = int(x)
        self.y = int(y)
        self.width = int(width)
        self.height = int(height)
        self.side = int(side)
        self.confidence = float(confidence)


class VisualCommand:
    """MSPM0 下发给 K230 的视觉工作命令。"""

    __slots__ = ("mode", "target_digit", "route_region", "epoch")

    def __init__(self, mode, target_digit, route_region, epoch):
        self.mode = int(mode)
        self.target_digit = int(target_digit)
        self.route_region = int(route_region)
        self.epoch = int(epoch)


def _as_int(value):
    """把 MicroPython/NumPy 标量安全转换为整数。"""
    return int(value)


def _as_float(value):
    """把 MicroPython/NumPy 标量安全转换为浮点数。"""
    return float(value)


def normalize_yolo_result(result, labels):
    """将 YOLO 三数组结果规范化为 `(box, class_id, score)` 列表。"""
    if result is None or len(result) < 3:
        return []

    boxes = result[0]
    class_ids = result[1]
    scores = result[2]
    if len(boxes) != len(class_ids) or len(boxes) != len(scores):
        return []

    detections = []
    for index in range(len(boxes)):
        box = boxes[index]
        if box is None or len(box) < 4:
            continue
        try:
            class_id = _as_int(class_ids[index])
            score = _as_float(scores[index])
        except (TypeError, ValueError, IndexError):
            continue
        if class_id < 0 or class_id >= len(labels):
            continue
        detections.append((box, class_id, score))
    return detections


def clamp_box_xyxy(box, frame_width=IMAGE_WIDTH, frame_height=IMAGE_HEIGHT):
    """把 YOLO 返回的 xyxy 框裁剪到 AI 画面，并转换为 xywh。"""
    if box is None or len(box) < 4:
        return None
    try:
        x1 = int(box[0])
        y1 = int(box[1])
        x2 = int(box[2])
        y2 = int(box[3])
    except (TypeError, ValueError, IndexError):
        return None

    # CanMV YOLO11 的 detect 结果使用左上角和右下角坐标，不是宽高。
    if (x2 <= x1) or (y2 <= y1):
        return None
    if (x2 <= 0) or (y2 <= 0) or (x1 >= frame_width) or (y1 >= frame_height):
        return None

    x1 = max(0, min(frame_width - 1, x1))
    y1 = max(0, min(frame_height - 1, y1))
    x2 = max(x1 + 1, min(frame_width, x2))
    y2 = max(y1 + 1, min(frame_height, y2))
    return x1, y1, x2 - x1, y2 - y1


def side_from_center(center_x, frame_width=IMAGE_WIDTH, deadband=CENTER_DEADBAND_X):
    """根据 AI 框中心判断车体左、中、右方位。"""
    center = frame_width // 2
    if center_x < center - deadband:
        return SIDE_LEFT
    if center_x > center + deadband:
        return SIDE_RIGHT
    return SIDE_CENTER


def make_detections(result, labels, min_confidence, target_digit=None):
    """从 YOLO 结果生成排序后的有效数字检测列表。"""
    detections = []
    for box, class_id, score in normalize_yolo_result(result, labels):
        if score < min_confidence:
            continue
        clipped = clamp_box_xyxy(box)
        if clipped is None:
            continue
        digit = int(labels[class_id])
        if target_digit not in (None, 0) and digit != int(target_digit):
            continue
        x, y, width, height = clipped
        center_x = x + (width // 2)
        side = side_from_center(center_x)
        detections.append(
            DigitDetection(
                digit,
                x,
                y,
                width,
                height,
                side,
                score,
            )
        )

    detections.sort(key=lambda item: item.confidence, reverse=True)
    return detections


def is_digit_inference_due(now_ms, last_run_ms, interval_ms):
    """使用无符号差值判断数字推理是否到期，兼容毫秒计数回绕。"""
    return ((int(now_ms) - int(last_run_ms)) & 0xFFFFFFFF) >= int(interval_ms)


def parse_visual_command(line):
    """解析 `V,mode,target_digit,route_region,epoch` 命令。"""
    if line is None:
        return None
    text = str(line).strip()
    fields = text.split(",")
    if len(fields) != 5 or fields[0] != "V":
        return None
    try:
        mode = int(fields[1])
        target_digit = int(fields[2])
        route_region = int(fields[3])
        epoch = int(fields[4])
    except ValueError:
        return None

    if mode < VISUAL_MODE_OFF or mode > VISUAL_MODE_TARGET:
        return None
    if target_digit < 0 or target_digit > 8:
        return None
    if route_region < 0 or route_region > 7:
        return None
    if epoch < 0 or epoch > 255:
        return None
    return VisualCommand(mode, target_digit, route_region, epoch)


def format_digit_frame(
    valid,
    digit,
    x,
    y,
    width,
    height,
    side,
    confidence,
    flags,
):
    """生成紧凑 D 帧，字段使用整数避免超过 MSPM0 行缓冲。"""
    if not valid:
        digit = 0
        x = 0
        y = 0
        width = 0
        height = 0
        side = SIDE_CENTER
        confidence = 0
    message = "D,{},{},{},{},{},{},{},{},{}\r\n".format(
        int(bool(valid)),
        int(digit),
        int(x),
        int(y),
        int(width),
        int(height),
        int(side),
        max(0, min(100, int(confidence))),
        int(flags) & 0x1F,
    )
    return message


class DigitConsensus:
    """药房数字多帧投票器；锁定后不会被后续视觉帧改写。"""

    def __init__(self, history_size=5, required_votes=3, max_gap_ms=800):
        self.history_size = int(history_size)
        self.required_votes = int(required_votes)
        self.max_gap_ms = int(max_gap_ms)
        self.history = []
        self.candidate_digit = 0
        self.candidate_side = SIDE_CENTER
        self.candidate_count = 0
        self.confirmed = False
        self.locked_digit = 0
        self.locked_side = SIDE_CENTER
        self.locked_confidence = 0
        self.last_seen_ms = None
        self.epoch = None

    def reset(self, epoch=None):
        self.history = []
        self.candidate_digit = 0
        self.candidate_side = SIDE_CENTER
        self.candidate_count = 0
        self.confirmed = False
        self.locked_digit = 0
        self.locked_side = SIDE_CENTER
        self.locked_confidence = 0
        self.last_seen_ms = None
        self.epoch = epoch

    def set_epoch(self, epoch):
        epoch = int(epoch)
        if self.epoch != epoch:
            self.reset(epoch)

    def observe(self, detection, now_ms):
        """加入一次有效观察并返回当前投票状态。"""
        if detection is None:
            return self.status()
        if self.confirmed:
            return self.status()

        now_ms = int(now_ms)
        if self.last_seen_ms is not None:
            elapsed = (now_ms - self.last_seen_ms) & 0xFFFFFFFF
            if elapsed > self.max_gap_ms:
                self.history = []
        self.last_seen_ms = now_ms
        self.history.append(detection)
        if len(self.history) > self.history_size:
            self.history.pop(0)

        counts = {}
        score_sums = {}
        for item in self.history:
            counts[item.digit] = counts.get(item.digit, 0) + 1
            score_sums[item.digit] = score_sums.get(item.digit, 0.0) + item.confidence

        best_digit = 0
        best_count = 0
        best_score = -1.0
        for digit in counts:
            count = counts[digit]
            score = score_sums[digit]
            if count > best_count or (count == best_count and score > best_score):
                best_digit = digit
                best_count = count
                best_score = score

        self.candidate_digit = best_digit
        self.candidate_count = best_count
        for item in reversed(self.history):
            if item.digit == best_digit:
                self.candidate_side = item.side
                break

        if best_count >= self.required_votes:
            self.confirmed = True
            self.locked_digit = best_digit
            self.locked_side = self.candidate_side
            self.locked_confidence = int(
                max(0.0, min(1.0, best_score / float(best_count))) * 100.0
            )
        return self.status()

    def status(self):
        return {
            "candidate_digit": self.candidate_digit,
            "candidate_side": self.candidate_side,
            "candidate_count": self.candidate_count,
            "confirmed": self.confirmed,
            "locked_digit": self.locked_digit,
            "locked_side": self.locked_side,
            "locked_confidence": self.locked_confidence,
        }
