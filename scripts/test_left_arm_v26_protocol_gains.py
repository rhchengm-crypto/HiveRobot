import ast
import unittest
from pathlib import Path


SOURCE = Path(__file__).with_name("left_arm_v2_6.py")


class LegacyMitGainMigrationTests(unittest.TestCase):
    def test_all_static_kd_values_fit_the_mit_protocol(self):
        tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
        violations = []
        for node in ast.walk(tree):
            if not isinstance(node, ast.Dict):
                continue
            for key, value in zip(node.keys, node.values):
                if (
                    isinstance(key, ast.Constant)
                    and key.value == "kd"
                    and isinstance(value, ast.Constant)
                    and isinstance(value.value, (int, float))
                    and not 0.0 <= float(value.value) <= 5.0
                ):
                    violations.append((node.lineno, value.value))
        self.assertEqual(violations, [])

    def test_home_gains_preserve_pre_fix_wire_damping(self):
        source = SOURCE.read_text(encoding="utf-8")
        tree = ast.parse(source)
        assignments = {
            node.targets[0].id: ast.literal_eval(node.value)
            for node in tree.body
            if isinstance(node, ast.Assign)
            and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)
            and node.targets[0].id in {"HOME_GAINS", "COUPLED_HOME_MOVE_GAINS"}
        }
        self.assertEqual(assignments["HOME_GAINS"]["arm_roll"]["kd"], 1.0)
        coupled = assignments["COUPLED_HOME_MOVE_GAINS"]
        self.assertEqual(coupled["elbow"]["kd"], 1.5)
        self.assertEqual(coupled["shoulder_front"]["kd"], 2.0)
        self.assertEqual(coupled["arm_roll"]["kd"], 1.0)


if __name__ == "__main__":
    unittest.main()
