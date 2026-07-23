"""数字识别纯逻辑、方位、投票和协议格式测试。"""

import pathlib
import sys
import unittest


K230_DIR = pathlib.Path(__file__).resolve().parents[1]
if str(K230_DIR) not in sys.path:
    sys.path.insert(0, str(K230_DIR))

from digit_logic import (  # noqa: E402
    DIGIT_FLAG_CONSENSUS,
    DIGIT_FLAG_LOCKED,
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
    SIDE_CENTER,
    SIDE_LEFT,
    SIDE_RIGHT,
    DigitConsensus,
    DigitDetection,
    clamp_box_xyxy,
    format_digit_frame,
    is_digit_inference_due,
    make_detections,
    parse_visual_command,
    side_from_center,
)


class DigitLogicTest(unittest.TestCase):
    def test_box_is_clipped_to_ai_frame(self):
        self.assertEqual(
            clamp_box_xyxy((-5, -2, 1000, 500)),
            (0, 0, IMAGE_WIDTH, IMAGE_HEIGHT),
        )
        self.assertIsNone(clamp_box_xyxy((1, 2, 0, 10)))

    def test_side_uses_corrected_ai_coordinates(self):
        self.assertEqual(side_from_center(100), SIDE_LEFT)
        self.assertEqual(side_from_center(320), SIDE_CENTER)
        self.assertEqual(side_from_center(540), SIDE_RIGHT)

    def test_yolo_results_keep_class_ids_and_filter_target(self):
        result = (
            [[20, 30, 80, 80], [480, 30, 560, 110]],
            [0, 5],
            [0.91, 0.95],
        )
        detections = make_detections(result, ["1", "2", "3", "4", "5", "6"], 0.60)
        self.assertEqual([item.digit for item in detections], [6, 1])
        self.assertEqual(detections[0].side, SIDE_RIGHT)
        self.assertEqual(
            (detections[0].x, detections[0].y,
             detections[0].width, detections[0].height),
            (480, 30, 80, 80),
        )
        self.assertEqual(
            [item.digit for item in make_detections(result, ["1", "2", "3", "4", "5", "6"], 0.60, 1)],
            [1],
        )

    def test_invalid_result_shape_is_ignored(self):
        self.assertEqual(make_detections(([], [0], []), ["1"], 0.5), [])
        self.assertEqual(make_detections(None, ["1"], 0.5), [])

    def test_consensus_locks_after_three_of_five(self):
        consensus = DigitConsensus(history_size=5, required_votes=3)
        for index in range(2):
            status = consensus.observe(
                DigitDetection(6, 500, 40, 70, 70, SIDE_RIGHT, 0.90),
                index * 100,
            )
            self.assertFalse(status["confirmed"])

        status = consensus.observe(
            DigitDetection(6, 500, 40, 70, 70, SIDE_RIGHT, 0.88),
            200,
        )
        self.assertTrue(status["confirmed"])
        self.assertEqual(status["locked_digit"], 6)
        self.assertEqual(status["locked_side"], SIDE_RIGHT)

        # 锁定后冲突数字不能覆盖原目标。
        status = consensus.observe(
            DigitDetection(2, 20, 40, 70, 70, SIDE_LEFT, 0.99),
            300,
        )
        self.assertEqual(status["locked_digit"], 6)

    def test_epoch_reset_allows_new_target(self):
        consensus = DigitConsensus(required_votes=1)
        consensus.set_epoch(1)
        consensus.observe(DigitDetection(3, 20, 20, 30, 30, SIDE_LEFT, 0.9), 0)
        self.assertEqual(consensus.status()["locked_digit"], 3)
        consensus.set_epoch(2)
        self.assertFalse(consensus.status()["confirmed"])
        self.assertEqual(consensus.status()["locked_digit"], 0)

    def test_visual_command_parser(self):
        command = parse_visual_command("V,2,6,3,17\r\n")
        self.assertIsNotNone(command)
        self.assertEqual((command.mode, command.target_digit, command.route_region, command.epoch), (2, 6, 3, 17))
        self.assertIsNone(parse_visual_command("V,9,6,3,17"))
        self.assertIsNone(parse_visual_command("V,2,9,3,17"))

    def test_digit_frame_stays_within_protocol_buffer(self):
        frame = format_digit_frame(
            True, 8, 639, 359, 640, 360, SIDE_RIGHT, 100,
            DIGIT_FLAG_CONSENSUS | DIGIT_FLAG_LOCKED,
        )
        self.assertTrue(frame.endswith("\r\n"))
        self.assertLessEqual(len(frame) - 2, 31)

        invalid = format_digit_frame(False, 8, 639, 359, 640, 360, SIDE_RIGHT, 100, 0)
        self.assertEqual(invalid, "D,0,0,0,0,0,0,0,0,0\r\n")

    def test_scheduler_handles_due_and_not_due(self):
        self.assertFalse(is_digit_inference_due(100, 0, 200))
        self.assertTrue(is_digit_inference_due(200, 0, 200))
        self.assertTrue(is_digit_inference_due(10, 0xFFFFFF00, 200))


if __name__ == "__main__":
    unittest.main()
