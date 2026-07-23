"""K230 红色中心线与路口方向识别。

传统视觉通道固定使用 320×240 RGB565。模块只依赖传入图像对象提供的
``find_blobs`` 接口，拟合和评分函数可直接在 CPython 单元测试中运行。
"""

import math


IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
IMAGE_CENTER_X = 160
CONTROL_Y = 204

# LAB 起始阈值仅供首次上板使用，必须按实际胶带、相机和赛场光照重新标定。
RED_THRESHOLDS = [(20, 100, 10, 127, 5, 127)]

# ROI 按由近到远的顺序排列。近场权重更高，避免远处路口横线主导转向。
TRACK_ROIS = (
    (0, 192, 320, 48, 5.0),
    (0, 150, 320, 42, 4.0),
    (0, 112, 320, 38, 3.0),
    (0, 76, 320, 36, 2.0),
    (0, 40, 320, 36, 1.0),
)

MIN_BLOB_PIXELS = 12
MIN_BLOB_AREA = 18
MAX_TRACK_JUMP_PX = 72
HORIZONTAL_BLOB_RATIO = 2.2
OUTLIER_REJECT_PX = 12.0
MIN_VALID_POINTS = 3
MIN_VERTICAL_SPAN_PX = 58
MAX_FIT_RMS_PX = 12.0
MIN_VALID_QUALITY = 45
MAX_ABS_ANGLE_DEG = 70.0
EXPECTED_PIXELS_PER_ROI = 90.0

# 路口横向分支至少延伸到中心线两侧该距离，才视为真实方向。
JUNCTION_SEARCH_ROI = (0, 54, 320, 112)
JUNCTION_MIN_PIXELS = 35
JUNCTION_MIN_WIDTH = 84
JUNCTION_SIDE_EXTENT_PX = 58
JUNCTION_CENTER_GATE_PX = 24
JUNCTION_FORWARD_HALF_WIDTH = 22
JUNCTION_FORWARD_HEIGHT = 54
JUNCTION_FORWARD_MIN_PIXELS = 18

DIRECTION_LEFT = 0x01
DIRECTION_FRONT = 0x02
DIRECTION_RIGHT = 0x04


class LineResult:
    """单帧红线识别结果，字段均可直接编码为紧凑 UART 整数。"""

    __slots__ = (
        "valid",
        "error_x",
        "angle_d10",
        "quality",
        "direction_mask",
        "point_count",
        "fit_rms",
    )

    def __init__(
        self,
        valid=False,
        error_x=0,
        angle_d10=0,
        quality=0,
        direction_mask=0,
        point_count=0,
        fit_rms=0.0,
    ):
        self.valid = bool(valid)
        self.error_x = int(error_x)
        self.angle_d10 = int(angle_d10)
        self.quality = int(quality)
        self.direction_mask = int(direction_mask)
        self.point_count = int(point_count)
        self.fit_rms = float(fit_rms)


class RedLineDetector:
    """维护帧间中心预测，并从 RGB565 图像输出红线和路口结果。"""

    def __init__(self, thresholds=None, center_x=IMAGE_CENTER_X):
        self.thresholds = RED_THRESHOLDS if thresholds is None else thresholds
        self.center_x = int(center_x)
        self.previous_x = int(center_x)

    def reset(self):
        """清除帧间预测，下一帧重新从标定中心搜索。"""
        self.previous_x = self.center_x

    def detect(self, img):
        """识别一帧红线；图像尺寸不匹配时返回无效结果。"""
        if img is None or img.width() != IMAGE_WIDTH or img.height() != IMAGE_HEIGHT:
            return LineResult()

        points = self._collect_track_points(img)
        analysis = analyze_track_points(points, self.center_x, CONTROL_Y)

        if not analysis.valid:
            return analysis

        slope, intercept, _ = fit_weighted_line(points)
        analysis.direction_mask = self._detect_direction_mask(
            img,
            slope,
            intercept,
        )
        self.previous_x = _clip_int(
            self.center_x + analysis.error_x,
            0,
            IMAGE_WIDTH - 1,
        )
        return analysis

    def _collect_track_points(self, img):
        points = []
        expected_x = self.previous_x

        for roi_index, roi_config in enumerate(TRACK_ROIS):
            x, y, width, height, weight = roi_config
            blobs = img.find_blobs(
                self.thresholds,
                roi=(x, y, width, height),
                pixels_threshold=MIN_BLOB_PIXELS,
                area_threshold=MIN_BLOB_AREA,
                merge=True,
            )

            predicted_x = _predict_next_x(points, y + height // 2, expected_x)
            selected = _select_track_blob(blobs, predicted_x)
            if selected is None:
                continue

            point_x, blob = selected
            point_y = blob.cy()
            points.append(
                (
                    float(point_x),
                    float(point_y),
                    float(weight),
                    float(blob.pixels()),
                    roi_index,
                )
            )
            expected_x = point_x

        return reject_largest_outlier(points)

    def _detect_direction_mask(self, img, slope, intercept):
        blobs = img.find_blobs(
            self.thresholds,
            roi=JUNCTION_SEARCH_ROI,
            pixels_threshold=JUNCTION_MIN_PIXELS,
            area_threshold=JUNCTION_MIN_PIXELS,
            merge=True,
        )

        best_blob = None
        best_score = None
        best_center_x = self.center_x

        for blob in blobs:
            rect_x, rect_y, rect_width, rect_height = blob.rect()
            if rect_width < JUNCTION_MIN_WIDTH:
                continue

            center_y = blob.cy()
            predicted_center_x = _clip_int(
                int(round(slope * center_y + intercept)),
                0,
                IMAGE_WIDTH - 1,
            )
            rect_right = rect_x + rect_width
            if not (
                rect_x - JUNCTION_CENTER_GATE_PX
                <= predicted_center_x
                <= rect_right + JUNCTION_CENTER_GATE_PX
            ):
                continue

            score = blob.pixels() + rect_width * 2 - abs(blob.cx() - predicted_center_x)
            if best_score is None or score > best_score:
                best_blob = blob
                best_score = score
                best_center_x = predicted_center_x

        # 没有检测到横向分支时，只要主线有效就按普通直线处理。
        if best_blob is None:
            return DIRECTION_FRONT

        rect_x, _, rect_width, _ = best_blob.rect()
        rect_right = rect_x + rect_width
        mask = 0

        if best_center_x - rect_x >= JUNCTION_SIDE_EXTENT_PX:
            mask |= DIRECTION_LEFT
        if rect_right - best_center_x >= JUNCTION_SIDE_EXTENT_PX:
            mask |= DIRECTION_RIGHT

        if _has_forward_line(
            img,
            self.thresholds,
            best_blob.cy(),
            best_center_x,
        ):
            mask |= DIRECTION_FRONT

        return mask


def _clip(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def _clip_int(value, minimum, maximum):
    return int(_clip(value, minimum, maximum))


def _blob_density(blob):
    try:
        return float(blob.density())
    except Exception:
        rect = blob.rect()
        area = rect[2] * rect[3]
        return float(blob.pixels()) / float(area) if area > 0 else 0.0


def _select_track_blob(blobs, expected_x):
    best = None
    best_score = None

    for blob in blobs:
        rect_x, _, rect_width, rect_height = blob.rect()
        distance = abs(blob.cx() - expected_x)
        if distance > MAX_TRACK_JUMP_PX and not (
            rect_x <= expected_x <= rect_x + rect_width
        ):
            continue

        # 横向路口连通域跨过预测中心时，沿预测中心继续取点，避免质心跳向支路。
        is_horizontal = rect_width > max(1, rect_height) * HORIZONTAL_BLOB_RATIO
        if is_horizontal and rect_x <= expected_x <= rect_x + rect_width:
            point_x = expected_x
            distance = 0
        else:
            point_x = blob.cx()

        density_bonus = int(_blob_density(blob) * 40.0)
        horizontal_penalty = max(0, rect_width - rect_height * 3)
        score = (
            blob.pixels()
            + density_bonus
            - distance * 3
            - horizontal_penalty
        )

        if best_score is None or score > best_score:
            best = (int(point_x), blob)
            best_score = score

    return best


def _predict_next_x(points, target_y, fallback_x):
    if len(points) < 2:
        return int(fallback_x if not points else points[-1][0])

    x1, y1 = points[-1][0], points[-1][1]
    x2, y2 = points[-2][0], points[-2][1]
    delta_y = y1 - y2
    if abs(delta_y) < 1.0:
        return int(x1)

    slope = (x1 - x2) / delta_y
    predicted = x1 + slope * (target_y - y1)
    return _clip_int(round(predicted), 0, IMAGE_WIDTH - 1)


def fit_weighted_line(points):
    """对 ``(x, y, weight, ...)`` 点集拟合 ``x=a*y+b``。"""
    if len(points) < 2:
        return 0.0, 0.0, 1.0e9

    sum_weight = 0.0
    sum_x = 0.0
    sum_y = 0.0

    for point in points:
        weight = float(point[2])
        sum_weight += weight
        sum_x += weight * float(point[0])
        sum_y += weight * float(point[1])

    if sum_weight <= 0.0:
        return 0.0, 0.0, 1.0e9

    mean_x = sum_x / sum_weight
    mean_y = sum_y / sum_weight
    numerator = 0.0
    denominator = 0.0

    for point in points:
        weight = float(point[2])
        centered_y = float(point[1]) - mean_y
        numerator += weight * centered_y * (float(point[0]) - mean_x)
        denominator += weight * centered_y * centered_y

    if abs(denominator) < 0.001:
        return 0.0, mean_x, 1.0e9

    slope = numerator / denominator
    intercept = mean_x - slope * mean_y
    squared_error = 0.0

    for point in points:
        residual = float(point[0]) - (slope * float(point[1]) + intercept)
        squared_error += float(point[2]) * residual * residual

    rms = math.sqrt(squared_error / sum_weight)
    return slope, intercept, rms


def reject_largest_outlier(points):
    """点数充足时剔除一次最大残差点，避免孤立红块破坏拟合。"""
    if len(points) < 4:
        return points

    slope, intercept, _ = fit_weighted_line(points)
    worst_index = -1
    worst_residual = 0.0

    for index, point in enumerate(points):
        residual = abs(float(point[0]) - (slope * float(point[1]) + intercept))
        if residual > worst_residual:
            worst_index = index
            worst_residual = residual

    if worst_index < 0 or worst_residual <= OUTLIER_REJECT_PX:
        return points

    return points[:worst_index] + points[worst_index + 1:]


def analyze_track_points(points, center_x=IMAGE_CENTER_X, control_y=CONTROL_Y):
    """把已提取中心点转换为控制误差、角度和确定性质量评分。"""
    point_count = len(points)
    if point_count < 2:
        return LineResult(point_count=point_count)

    slope, intercept, rms = fit_weighted_line(points)
    minimum_y = min(point[1] for point in points)
    maximum_y = max(point[1] for point in points)
    vertical_span = maximum_y - minimum_y
    accepted_weight = sum(point[2] for point in points)
    total_weight = sum(roi[4] for roi in TRACK_ROIS)
    coverage_score = _clip(accepted_weight / total_weight, 0.0, 1.0)
    fit_score = _clip(1.0 - rms / MAX_FIT_RMS_PX, 0.0, 1.0)
    span_score = _clip(vertical_span / 120.0, 0.0, 1.0)

    support_sum = 0.0
    for point in points:
        support_sum += _clip(point[3] / EXPECTED_PIXELS_PER_ROI, 0.0, 1.0)
    support_score = support_sum / point_count

    quality = int(round(100.0 * (
        0.40 * coverage_score
        + 0.30 * fit_score
        + 0.20 * span_score
        + 0.10 * support_score
    )))

    line_x = slope * float(control_y) + intercept
    error_x = _clip_int(round(line_x - center_x), -160, 160)
    angle_deg = -math.degrees(math.atan(slope))
    angle_d10 = _clip_int(round(angle_deg * 10.0), -900, 900)
    near_hit = any(point[4] <= 1 for point in points)

    valid = (
        point_count >= MIN_VALID_POINTS
        and near_hit
        and vertical_span >= MIN_VERTICAL_SPAN_PX
        and rms <= MAX_FIT_RMS_PX
        and quality >= MIN_VALID_QUALITY
        and abs(angle_deg) <= MAX_ABS_ANGLE_DEG
    )

    if not valid:
        return LineResult(
            valid=False,
            quality=quality,
            point_count=point_count,
            fit_rms=rms,
        )

    return LineResult(
        valid=True,
        error_x=error_x,
        angle_d10=angle_d10,
        quality=quality,
        direction_mask=DIRECTION_FRONT,
        point_count=point_count,
        fit_rms=rms,
    )


def direction_mask_from_evidence(left, front, right):
    """供单元测试和路口聚合复用的方向位编码。"""
    mask = 0
    if left:
        mask |= DIRECTION_LEFT
    if front:
        mask |= DIRECTION_FRONT
    if right:
        mask |= DIRECTION_RIGHT
    return mask


def _has_forward_line(img, thresholds, junction_y, center_x):
    top = max(0, int(junction_y) - JUNCTION_FORWARD_HEIGHT)
    height = int(junction_y) - top
    if height <= 4:
        return False

    left = _clip_int(
        int(center_x) - JUNCTION_FORWARD_HALF_WIDTH,
        0,
        IMAGE_WIDTH - 1,
    )
    width = min(
        JUNCTION_FORWARD_HALF_WIDTH * 2,
        IMAGE_WIDTH - left,
    )
    blobs = img.find_blobs(
        thresholds,
        roi=(left, top, width, height),
        pixels_threshold=JUNCTION_FORWARD_MIN_PIXELS,
        area_threshold=JUNCTION_FORWARD_MIN_PIXELS,
        merge=True,
    )
    return len(blobs) > 0
