"""Record user-confirmed reference and compare board-plane coordinates only."""
import json
from pathlib import Path
import cv2
import numpy as np

out = Path(__file__).resolve().parent
pose = json.loads((out / 'restored_board_camera_registration.json').read_text())
camera = json.loads((out / 'camera_board_registration.json').read_text())
K = np.array(camera['camera_matrix'])
R = np.array(pose['board_to_camera_rotation'])
t = np.array(pose['board_to_camera_translation_mm'])
im = cv2.imdecode(np.fromfile(out / 'board_ruler_after_home.jpg', np.uint8), cv2.IMREAD_COLOR)
points = np.array([[113.8, 190.3, 0], [137.5, 192.5, 0.]])
p = (points @ R.T + t) @ K.T
p = p[:, :2] / p[:, 2:]
for xy, label, color, dy in zip(p, ['Specified XY (board plane)', 'C4 center (board plane)'], [(0, 220, 255), (255, 180, 0)], [-24, 33]):
    center = tuple(np.rint(xy).astype(int))
    cv2.drawMarker(im, center, color, cv2.MARKER_CROSS, 12, 2)
    cv2.putText(im, label, (max(5, center[0]-95), center[1]+dy), cv2.FONT_HERSHEY_SIMPLEX, .4, color, 1, cv2.LINE_AA)
cv2.imencode('.png', im)[1].tofile(out / 'board_reference_coordinates.png')
report = {
    'user_confirmed': {'ruler_right_end_near_C4': '0 cm', 'purpose': 'stationary board reference only; not grasp target'},
    'requested_target_mm': [113.8, 190.3, 50.2],
    'C4_center_on_board_mm': [137.5, 192.5, 0],
    'target_minus_C4_XY_mm': (points[0, :2]-points[1, :2]).tolist(),
    'annotation': 'Both crosses are projected onto the board surface, not the elevated TCP.',
    'limitations': ['Ruler zero is user-confirmed; tick spacing is not calibrated from this image.', 'Uses prior restored-board camera registration; does not update production board transform.'],
    'robot_state_verified': 'Home completed; post-home hold completed; no motor command process running',
    'production_changes': False,
    'new_descent_executed_after_home': False,
}
(out / 'ruler_reference_confirmation.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report))
