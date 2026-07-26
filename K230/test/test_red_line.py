"""红线拟合和路口位编码的主机单元测试。"""

import math
import pathlib
import sys
import unittest


K230_DIR = pathlib.Path(__file__).resolve().parents[1]
if str(K230_DIR) not in sys.path:
    sys.path.insert(0, str(K230_DIR))

from red_line import (  # noqa: E402
    DIRECTION_FRONT,
    DIRECTION_LEFT,
    DIRECTION_RIGHT,
    JUNCTION_FORWARD_HALF_WIDTH,
    JUNCTION_FORWARD_HEIGHT,
    JUNCTION_FORWARD_MIN_REACH_PX,
    JUNCTION_SEARCH_ROI,
    RedLineDetector,
    TRACK_ROIS,
    analyze_track_points,
    direction_mask_from_evidence,
    fit_weighted_line,
    reject_largest_outlier,
)


def point(x, y, weight, pixels=120.0, roi_index=0):
    return (float(x), float(y), float(weight), float(pixels), int(roi_index))


class FakeBlob:
    def __init__(self, rect, pixels=120, center=None):
        self._rect = rect
        self._pixels = pixels
        self._center = center

    def rect(self):
        return self._rect

    def pixels(self):
        return self._pixels

    def cx(self):
        if self._center is not None:
            return self._center[0]
        return self._rect[0] + self._rect[2] // 2

    def cy(self):
        if self._center is not None:
            return self._center[1]
        return self._rect[1] + self._rect[3] // 2

    def density(self):
        area = self._rect[2] * self._rect[3]
        return self._pixels / area


class FakeImage:
    def __init__(self, responses):
        self.responses = responses

    def width(self):
        return 320

    def height(self):
        return 240

    def find_blobs(self, _thresholds, roi=None, **_kwargs):
        return self.responses.get(tuple(roi), [])


def make_fake_image(junction_blob=None, forward_blob=None):
    responses = {}
    for x, y, width, height, _weight in TRACK_ROIS:
        responses[(x, y, width, height)] = [
            FakeBlob((154, y, 12, height), center=(160, y + height // 2))
        ]

    responses[JUNCTION_SEARCH_ROI] = [] if junction_blob is None else [junction_blob]
    if junction_blob is not None:
        center_x = junction_blob.cx()
        top = junction_blob.cy() - JUNCTION_FORWARD_HEIGHT
        left = center_x - JUNCTION_FORWARD_HALF_WIDTH
        width = JUNCTION_FORWARD_HALF_WIDTH * 2

        # 真实前向 ROI 会裁入横线自身的上半部分，旧逻辑会因此把 T 字误判为十字。
        pollution_height = max(4, JUNCTION_FORWARD_MIN_REACH_PX // 3)
        pollution = FakeBlob(
            (
                left,
                junction_blob.cy() - pollution_height,
                width,
                pollution_height,
            ),
            pixels=width * pollution_height,
        )
        forward_responses = [pollution]
        if forward_blob is not None:
            forward_responses.append(forward_blob)
        responses[(left, top, width, JUNCTION_FORWARD_HEIGHT)] = forward_responses
    return FakeImage(responses)


class RedLineMathTest(unittest.TestCase):
    def test_centered_vertical_line(self):
        points = [
            point(160, 216, 5, roi_index=0),
            point(160, 170, 4, roi_index=1),
            point(160, 130, 3, roi_index=2),
            point(160, 92, 2, roi_index=3),
            point(160, 56, 1, roi_index=4),
        ]

        result = analyze_track_points(points)

        self.assertTrue(result.valid)
        self.assertEqual(result.error_x, 0)
        self.assertEqual(result.angle_d10, 0)
        self.assertGreaterEqual(result.quality, 90)

    def test_offset_line_keeps_error_sign(self):
        right_points = [
            point(180, 216, 5, roi_index=0),
            point(180, 170, 4, roi_index=1),
            point(180, 130, 3, roi_index=2),
        ]
        left_points = [
            point(138, 216, 5, roi_index=0),
            point(138, 170, 4, roi_index=1),
            point(138, 130, 3, roi_index=2),
        ]

        self.assertGreater(analyze_track_points(right_points).error_x, 0)
        self.assertLess(analyze_track_points(left_points).error_x, 0)

    def test_line_extending_right_ahead_has_positive_angle(self):
        # 图像上方代表前方；上方 x 更大时，车辆需要向右转。
        points = [
            point(150, 216, 5, roi_index=0),
            point(158, 170, 4, roi_index=1),
            point(166, 130, 3, roi_index=2),
            point(174, 92, 2, roi_index=3),
        ]

        result = analyze_track_points(points)

        self.assertTrue(result.valid)
        self.assertGreater(result.angle_d10, 0)

    def test_insufficient_points_are_invalid(self):
        result = analyze_track_points([
            point(160, 216, 5, roi_index=0),
            point(160, 170, 4, roi_index=1),
        ])

        self.assertFalse(result.valid)
        self.assertEqual(result.error_x, 0)
        self.assertEqual(result.angle_d10, 0)

    def test_missing_near_roi_is_invalid(self):
        result = analyze_track_points([
            point(160, 130, 3, roi_index=2),
            point(160, 92, 2, roi_index=3),
            point(160, 56, 1, roi_index=4),
        ])

        self.assertFalse(result.valid)

    def test_largest_outlier_is_removed_once(self):
        points = [
            point(160, 216, 5, roi_index=0),
            point(160, 170, 4, roi_index=1),
            point(235, 130, 3, roi_index=2),
            point(160, 92, 2, roi_index=3),
            point(160, 56, 1, roi_index=4),
        ]

        filtered = reject_largest_outlier(points)
        slope, _, rms = fit_weighted_line(filtered)

        self.assertEqual(len(filtered), 4)
        self.assertAlmostEqual(slope, 0.0, places=3)
        self.assertLess(rms, 0.01)

    def test_direction_masks(self):
        self.assertEqual(
            direction_mask_from_evidence(False, True, False),
            DIRECTION_FRONT,
        )
        self.assertEqual(
            direction_mask_from_evidence(True, False, True),
            DIRECTION_LEFT | DIRECTION_RIGHT,
        )
        self.assertEqual(
            direction_mask_from_evidence(True, True, True),
            DIRECTION_LEFT | DIRECTION_FRONT | DIRECTION_RIGHT,
        )

    def test_detector_reports_straight_t_and_cross(self):
        detector = RedLineDetector()
        straight = detector.detect(make_fake_image())
        self.assertTrue(straight.valid)
        self.assertEqual(straight.direction_mask, DIRECTION_FRONT)

        # 使用较厚横线并让前向 ROI 包含横线污染，复现实车 T 字误判 M7。
        junction = FakeBlob((40, 96, 240, 18), pixels=420, center=(160, 105))
        t_result = detector.detect(make_fake_image(junction_blob=junction))
        self.assertTrue(t_result.valid)
        self.assertEqual(t_result.direction_mask, DIRECTION_LEFT | DIRECTION_RIGHT)

        forward = FakeBlob((154, 65, 12, 40), pixels=80, center=(160, 85))
        cross_result = detector.detect(
            make_fake_image(junction_blob=junction, forward_blob=forward)
        )
        self.assertTrue(cross_result.valid)
        self.assertEqual(
            cross_result.direction_mask,
            DIRECTION_LEFT | DIRECTION_FRONT | DIRECTION_RIGHT,
        )

    def test_isolated_invalid_frames_hold_last_reliable_line(self):
        detector = RedLineDetector()
        reliable = detector.detect(make_fake_image())
        self.assertTrue(reliable.valid)

        # 相同场景中偶发一两个 ROI 全部未成 blob 时，短时保持上次可靠控制量。
        first_miss = detector.detect(FakeImage({}))
        second_miss = detector.detect(FakeImage({}))
        third_miss = detector.detect(FakeImage({}))

        self.assertTrue(first_miss.valid)
        self.assertTrue(second_miss.valid)
        self.assertEqual(first_miss.error_x, reliable.error_x)
        self.assertEqual(second_miss.direction_mask, reliable.direction_mask)
        self.assertLess(first_miss.quality, reliable.quality)
        self.assertLessEqual(second_miss.quality, first_miss.quality)
        self.assertFalse(third_miss.valid)

    def test_fit_matches_known_slope(self):
        points = [
            point(0.2 * y + 100.0, y, weight, roi_index=index)
            for index, (y, weight) in enumerate(
                ((216, 5), (170, 4), (130, 3), (92, 2))
            )
        ]

        slope, intercept, rms = fit_weighted_line(points)

        self.assertTrue(math.isclose(slope, 0.2, abs_tol=1e-6))
        self.assertTrue(math.isclose(intercept, 100.0, abs_tol=1e-6))
        self.assertLess(rms, 1e-6)


if __name__ == "__main__":
    unittest.main()
