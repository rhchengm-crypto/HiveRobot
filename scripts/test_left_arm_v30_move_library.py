import sys
import unittest
from unittest.mock import patch

import left_arm_v2_8_move_library as v28
import left_arm_v3_0_move_library as v30


class V30MoveAdapterTests(unittest.TestCase):
    def test_c4_bishop_temporarily_uses_c4_source(self):
        original = v28.PLACEMENT1_MOVE_NAME
        observed = []

        def capture(args):
            observed.append((args.name, v28.PLACEMENT1_MOVE_NAME))

        argv = [
            "left_arm_v3_0_move_library.py", "replay-move", "--name", "white_knight_c4",
            "--white-bishop-placement",
        ]
        with patch.object(sys, "argv", argv), patch.object(v28, "replay_with_local_bias", capture):
            v30.main()
        self.assertEqual(observed, [("white_knight_c4", v28.PLACEMENT_C4_MOVE_NAME)])
        self.assertEqual(v28.PLACEMENT1_MOVE_NAME, original)

    def test_c4_rook_uses_c4_source_and_a1_destination(self):
        original_source = v28.PLACEMENT1_MOVE_NAME
        original_destination = v28.WHITE_BISHOP_PLACE_MOVE_NAME
        observed = []

        def capture(args):
            observed.append((
                args.name,
                args.white_bishop_placement,
                v28.PLACEMENT1_MOVE_NAME,
                v28.WHITE_BISHOP_PLACE_MOVE_NAME,
            ))

        argv = [
            "left_arm_v3_0_move_library.py", "replay-move", "--name", "white_knight_c4",
            "--white-rook-place-a1",
        ]
        with patch.object(sys, "argv", argv), patch.object(v28, "replay_with_local_bias", capture):
            v30.main()
        self.assertEqual(observed, [(
            "white_knight_c4", True, v28.PLACEMENT_C4_MOVE_NAME,
            "white_rook_place_A1",
        )])
        self.assertEqual(v28.PLACEMENT1_MOVE_NAME, original_source)
        self.assertEqual(v28.WHITE_BISHOP_PLACE_MOVE_NAME, original_destination)


if __name__ == "__main__":
    unittest.main()
