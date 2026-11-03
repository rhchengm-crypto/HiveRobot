"""Retain optical XY/normal; establish absolute plane height from observed touch.

This is a measured plane-datum calibration, not a runtime target-height offset.
It is limited to the unchanged CAD gripper opening used for the reported touch.
"""
import json,numpy as np,hashlib
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
c=json.loads((OUT/'board_model_failed_optical.json').read_text())
audit=json.loads((OUT/'contact_mesh_height_audit.json').read_text())
touch=next(r for r in audit if r['pose']=='hold_end');height=touch['minimum_CAD_gripper_height_mm']
assert 20<height<60,'Unexpected contact datum; do not silently extrapolate'
R=np.array(c['board_to_cad_rotation']);old_t=np.array(c['board_to_cad_translation_m']);t=old_t+R[:,2]*(height/1000)
c['board_to_cad_translation_m']=t.tolist();c['build']='seven-axis-board-grasp-v24-optical-normal-touch-height'
c['registration_status']='optical lateral/normal registration plus user-confirmed fingertip touch plane datum; pending empty-load validation'
c['independently_validated']=False
c['touch_plane_datum']={
 'source':'user-confirmed light fingertip contact, no gripper close or new motion used for calibration',
 'source_log':'arm_run_1793680858524_14500.txt',
 'contact_pose_source':'same-serial model hold final feedback',
 'CAD_envelope':'part7 OmniPicker vertex envelope in approved revised STEP',
 'minimum_vertex_before_calibration_board_mm':touch['lowest_CAD_vertex_board_mm'],
 'plane_origin_normal_change_mm':height,
 'optical_plane_translation_before_touch_m':old_t.tolist(),
 'optical_board_rotation_unchanged':True,
 'board_XY_coordinates_unchanged':True,
 'target_grip_height_unchanged_mm':50.2,
 'runtime_height_offset_mm':0.,
 'calibration_scope':'same gripper opening and local board surface; not yet validated for other openings or grasp contact centers',
 'assumptions':['user-reported light contact is on the board surface','lowest CAD gripper vertex represents the first touching fingertip at this unchanged opening','no board/camera/base movement since contact'],
 'physical_trial_complete':False,
}
assert abs(touch['minimum_CAD_gripper_height_mm']-np.dot(R[:,2],t-old_t)*1000)<1e-8
assert np.max(np.abs((t-old_t)@R[:,:2]))<1e-12
path=OUT/'left_arm_board_grasp_touch_candidate.json';path.write_text(json.dumps(c,indent=2))
report={'candidate':path.name,'board_normal_unchanged':True,'XY_projection_change_mm':((t-old_t)@R[:,:2]*1000).tolist(),'calibrated_touched_fingertip_height_mm':0.,'contact_TCP_height_after_plane_calibration_mm':touch['TCP_board_mm'][2]-height,'model_based_expected_minimum_gap_at_same_orientation_and_50_2_TCP_mm':50.2+(touch['lowest_point_relative_TCP_mm'][2]),'plane_origin_normal_change_mm':height,'motion_issued':False,'not_yet_physically_validated':True}
(OUT/'touch_off_candidate_audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
