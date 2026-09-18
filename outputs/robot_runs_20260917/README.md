# 2026-09-17 robot run evidence

This directory preserves the two user-provided White Rook A1 Web command outputs unchanged.
`index.json` records their source attachment IDs, timestamps, return codes and SHA-256 hashes.

The 14:14:50 multi-flow completed C4 pickup, Clearance, `white_rook_place_A1`, and Claw Home
with return code 0. The A1 placement phase remained `training incomplete`: shoulder_rotate,
arm_roll, wrist_side and wrist exceeded the nominal 0.5 degree threshold. The final Claw Home
completed and measured wrist drift of -0.415289 degrees (peak -0.437146 degrees).

The 16:51:18 run also returned 0. Its reported saved-move target and encoder/error data were
used to add `placement_anchors.white_rook:a1` to the tracked geometry library. The operator
approved this move as an empirical A1 placement anchor. It remains `validated=false` because
shoulder_side, shoulder_rotate, arm_roll and wrist exceeded the 0.5 degree training tolerance;
the largest absolute residual was arm_roll at 4.874091 degrees.

These files are runtime evidence, not a copy of the robot's private Saved Moves or adaptive
learning JSON. Do not overwrite robot runtime data with repository data.
