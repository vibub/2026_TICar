import math


ROI_MARGIN_RATIO = 0.20
RECT_THRESHOLD = 2000
MIN_RECT_AREA_RATIO = 0.20
MAX_RECT_AREA_RATIO = 1.40
MIN_ASPECT_RATIO = 0.50
MAX_ASPECT_RATIO = 2.00
CIRCLE_THRESHOLD = 800
ENABLE_CIRCLE_FALLBACK = False
DEBUG_GEOMETRY = True

_debug_call_count = 0
_debug_active = False


def _clip(value, minimum, maximum):
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def _expand_roi(box, image_width, image_height):
    x1, y1, x2, y2 = box
    width = x2 - x1
    height = y2 - y1

    margin_x = int(width * ROI_MARGIN_RATIO)
    margin_y = int(height * ROI_MARGIN_RATIO)

    left = _clip(x1 - margin_x, 0, image_width - 1)
    top = _clip(y1 - margin_y, 0, image_height - 1)
    right = _clip(x2 + margin_x, left + 1, image_width)
    bottom = _clip(y2 + margin_y, top + 1, image_height)

    return left, top, right - left, bottom - top


def _polygon_area(points):
    area = 0.0
    count = len(points)

    for index in range(count):
        next_index = (index + 1) % count
        area += points[index][0] * points[next_index][1]
        area -= points[next_index][0] * points[index][1]

    return abs(area) * 0.5


def _distance(point_a, point_b):
    dx = point_a[0] - point_b[0]
    dy = point_a[1] - point_b[1]
    return math.sqrt(dx * dx + dy * dy)


def _sort_corners(corners):
    center_x = 0.0
    center_y = 0.0

    for point in corners:
        center_x += point[0]
        center_y += point[1]

    center_x /= len(corners)
    center_y /= len(corners)

    return sorted(
        corners,
        key=lambda point: math.atan2(
            point[1] - center_y,
            point[0] - center_x
        )
    )


def _line_intersection(point_a, point_b, point_c, point_d):
    x1, y1 = point_a
    x2, y2 = point_b
    x3, y3 = point_c
    x4, y4 = point_d

    denominator = (
        (x1 - x2) * (y3 - y4) -
        (y1 - y2) * (x3 - x4)
    )

    if abs(denominator) < 0.001:
        return None

    determinant_a = x1 * y2 - y1 * x2
    determinant_b = x3 * y4 - y3 * x4

    center_x = (
        determinant_a * (x3 - x4) -
        (x1 - x2) * determinant_b
    ) / denominator

    center_y = (
        determinant_a * (y3 - y4) -
        (y1 - y2) * determinant_b
    ) / denominator

    return center_x, center_y


def _rectangle_candidate(rectangle, yolo_box, previous_center):
    corners = rectangle.corners()
    if corners is None or len(corners) != 4:
        return None

    points = []
    for corner in corners:
        points.append((float(corner[0]), float(corner[1])))

    points = _sort_corners(points)
    area = _polygon_area(points)

    yolo_width = yolo_box[2] - yolo_box[0]
    yolo_height = yolo_box[3] - yolo_box[1]
    yolo_area = yolo_width * yolo_height

    if yolo_area <= 0:
        return None

    area_ratio = area / yolo_area
    if area_ratio < MIN_RECT_AREA_RATIO or area_ratio > MAX_RECT_AREA_RATIO:
        return None

    edges = [
        _distance(points[0], points[1]),
        _distance(points[1], points[2]),
        _distance(points[2], points[3]),
        _distance(points[3], points[0])
    ]

    average_a = (edges[0] + edges[2]) * 0.5
    average_b = (edges[1] + edges[3]) * 0.5

    if average_a <= 1.0 or average_b <= 1.0:
        return None

    aspect_ratio = average_a / average_b
    if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
        inverse_ratio = 1.0 / aspect_ratio
        if inverse_ratio < MIN_ASPECT_RATIO or inverse_ratio > MAX_ASPECT_RATIO:
            return None

    center = _line_intersection(
        points[0],
        points[2],
        points[1],
        points[3]
    )
    if center is None:
        return None

    center_x, center_y = center
    min_x = min(point[0] for point in points)
    max_x = max(point[0] for point in points)
    min_y = min(point[1] for point in points)
    max_y = max(point[1] for point in points)

    if not (min_x <= center_x <= max_x and min_y <= center_y <= max_y):
        return None

    yolo_center_x = (yolo_box[0] + yolo_box[2]) * 0.5
    yolo_center_y = (yolo_box[1] + yolo_box[3]) * 0.5
    dx = center_x - yolo_center_x
    dy = center_y - yolo_center_y
    center_distance_squared = dx * dx + dy * dy

    magnitude = rectangle.magnitude()
    score = float(magnitude) - center_distance_squared * 4.0
    score += min(area_ratio, 1.0) * 1000.0

    if previous_center is not None:
        prev_dx = center_x - previous_center[0]
        prev_dy = center_y - previous_center[1]
        score -= (prev_dx * prev_dx + prev_dy * prev_dy) * 0.5

    return int(center_x), int(center_y), score, magnitude


def _find_rectangle_center(img, roi, yolo_box, previous_center):
    try:
        rectangles = img.find_rects(roi, RECT_THRESHOLD)
    except Exception as error:
        if _debug_active:
            print("GEOMETRY find_rects异常:", error)
        return None

    if _debug_active:
        print("GEOMETRY ROI:", roi, "rectangles:", len(rectangles))

    best_candidate = None

    for rectangle in rectangles:
        candidate = _rectangle_candidate(
            rectangle,
            yolo_box,
            previous_center
        )
        if _debug_active:
            print(
                "GEOMETRY RECT:",
                rectangle.corners(),
                "magnitude=", rectangle.magnitude(),
                "accepted=", candidate is not None
            )

        if candidate is None:
            continue

        if best_candidate is None or candidate[2] > best_candidate[2]:
            best_candidate = candidate

    if best_candidate is None:
        return None

    return (
        best_candidate[0],
        best_candidate[1],
        "RECT",
        int(best_candidate[3])
    )


def _find_concentric_center(img, roi, yolo_box):
    max_radius = max(8, min(roi[2], roi[3]) // 2)

    try:
        circles = img.find_circles(
            roi,
            2,
            2,
            CIRCLE_THRESHOLD,
            8,
            8,
            8,
            4,
            max_radius,
            2
        )
    except Exception as error:
        if _debug_active:
            print("GEOMETRY find_circles异常:", error)
        return None

    if _debug_active:
        print("GEOMETRY circles:", len(circles))

    if len(circles) < 2:
        return None

    center_tolerance = max(5, min(roi[2], roi[3]) // 20)
    yolo_center_x = (yolo_box[0] + yolo_box[2]) * 0.5
    yolo_center_y = (yolo_box[1] + yolo_box[3]) * 0.5
    best_group = None
    best_score = None

    for anchor in circles:
        group = []

        for circle in circles:
            dx = circle.x() - anchor.x()
            dy = circle.y() - anchor.y()

            if dx * dx + dy * dy <= center_tolerance * center_tolerance:
                group.append(circle)

        if len(group) < 2:
            continue

        minimum_radius = min(circle.r() for circle in group)
        maximum_radius = max(circle.r() for circle in group)
        if maximum_radius - minimum_radius < 4:
            continue

        weight_sum = 0.0
        center_x_sum = 0.0
        center_y_sum = 0.0

        for circle in group:
            weight = max(1, circle.magnitude())
            weight_sum += weight
            center_x_sum += circle.x() * weight
            center_y_sum += circle.y() * weight

        center_x = center_x_sum / weight_sum
        center_y = center_y_sum / weight_sum
        dx = center_x - yolo_center_x
        dy = center_y - yolo_center_y
        score = len(group) * 10000.0 + weight_sum - (dx * dx + dy * dy)

        if best_score is None or score > best_score:
            best_score = score
            best_group = (
                int(center_x),
                int(center_y),
                "CIRCLE",
                len(group)
            )

    return best_group


def locate_target_center(img, yolo_box, previous_center=None):
    global _debug_call_count, _debug_active

    _debug_call_count += 1
    _debug_active = DEBUG_GEOMETRY and (_debug_call_count % 10 == 0)

    image_width = img.width()
    image_height = img.height()
    roi = _expand_roi(yolo_box, image_width, image_height)

    rectangle_center = _find_rectangle_center(
        img,
        roi,
        yolo_box,
        previous_center
    )
    if rectangle_center is not None:
        return rectangle_center

    if ENABLE_CIRCLE_FALLBACK:
        circle_center = _find_concentric_center(img, roi, yolo_box)
        if circle_center is not None:
            return circle_center

    center_x = (yolo_box[0] + yolo_box[2]) // 2
    center_y = (yolo_box[1] + yolo_box[3]) // 2
    return center_x, center_y, "YOLO", 0
