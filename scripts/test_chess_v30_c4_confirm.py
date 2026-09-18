import time
import unittest

import chessboard_vision_v3_0_web_control as v30


class C4ConfirmedRoutingTests(unittest.TestCase):
    def test_route_only_bishop_knight_and_rook(self):
        bishop = v30.classify_c4_route("white_bishop")
        self.assertEqual(bishop["saved_move"], "white_knight_c4")
        self.assertEqual(bishop["flow_key"], "white_bishop_placement")
        knight = v30.classify_c4_route("black-knight")
        self.assertEqual(knight["saved_move"], "white_knight_c4")
        self.assertEqual(knight["flow_key"], "white_knight_place_b1")
        rook = v30.classify_c4_route("white_rook")
        self.assertEqual(rook["saved_move"], "white_knight_c4")
        self.assertEqual(rook["flow_key"], "white_rook_place_a1")
        self.assertEqual(rook["flow_label"], "white rook place_A1")
        for piece in ("white_pawn", "white_queen", "white_king", ""):
            self.assertIsNone(v30.classify_c4_route(piece))

    def test_confirmation_is_one_time(self):
        pending = v30.PendingC4Confirmations(ttl_seconds=10)
        token, _ = pending.issue(
            {"piece_class": "white_bishop", "confidence": 0.9},
            v30.classify_c4_route("white_bishop"),
        )
        self.assertEqual(pending.consume(token)["route"]["piece_type"], "bishop")
        with self.assertRaisesRegex(ValueError, "失效或已使用"):
            pending.consume(token)

    def test_confirmation_expires(self):
        pending = v30.PendingC4Confirmations(ttl_seconds=-1)
        token, _ = pending.issue(
            {"piece_class": "white_knight"},
            v30.classify_c4_route("white_knight"),
        )
        time.sleep(0.001)
        with self.assertRaisesRegex(ValueError, "失效或已使用"):
            pending.consume(token)

    def test_page_contains_explicit_confirmation_and_prediction(self):
        page = v30.ARM_HTML_PAGE
        self.assertIn("识别 C4 并确认执行", page)
        self.assertIn('id="c4ConfirmModal"', page)
        self.assertIn("识别正确，执行机械臂", page)
        self.assertIn("prediction_image_url", page)
        self.assertIn("/api/v3/c4/inspect", page)
        self.assertIn("/api/v3/c4/execute", page)
        self.assertIn("enableMotionGate').checked", page)

    def test_v30_has_separate_default_port(self):
        args = v30.build_parser().parse_args([])
        self.assertEqual(args.port, 8100)
        self.assertFalse(args.enable_execute)
        self.assertTrue(args.move_script.endswith("left_arm_v3_0_move_library.py"))


if __name__ == "__main__":
    unittest.main()
