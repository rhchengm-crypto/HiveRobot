import re
import unittest
from pathlib import Path


RUNTIME_WEB_FILES = (
    "chessboard_vision_v2_7_web_control.py",
    "chessboard_vision_v2_8_web_control.py",
    "chessboard_vision_v3_0_web_control.py",
    "left_arm_v2_6_web_control.py",
    "chess_crown_geometry_v2_8.py",
    "chess_piece_height_web.py",
    "chess_piece_height_samples.py",
    "chess_piece_height_model.py",
    "chess_piece_yolo_labels.py",
    "hp60c_stream_server.py",
)
CHINESE_CHARACTERS = re.compile(r"[\u3400-\u4dbf\u4e00-\u9fff\uf900-\ufaff]")
CHINESE_PUNCTUATION = re.compile(r"[，。：；！？（）【】「」]")


class EnglishWebUiTests(unittest.TestCase):
    def test_runtime_web_modules_have_no_chinese_ui_text(self):
        root = Path(__file__).resolve().parent
        for filename in RUNTIME_WEB_FILES:
            text = (root / filename).read_text(encoding="utf-8")
            with self.subTest(filename=filename):
                self.assertIsNone(CHINESE_CHARACTERS.search(text))
                self.assertIsNone(CHINESE_PUNCTUATION.search(text))
                self.assertNotIn('lang="zh', text.lower())

    def test_v3_confirmation_controls_are_english(self):
        from chessboard_vision_v3_0_web_control import ARM_HTML_PAGE

        self.assertIn("Inspect C4 and Confirm Execution", ARM_HTML_PAGE)
        self.assertIn("Confirm C4 Piece", ARM_HTML_PAGE)
        self.assertIn("Recognition Is Correct - Run Robot Arm", ARM_HTML_PAGE)


if __name__ == "__main__":
    unittest.main()
