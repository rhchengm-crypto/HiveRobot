"""Independent board-image registration; does not modify robot geometry."""
import cv2,json,numpy as np
from pathlib import Path
OUT=Path(__file__).resolve().parent
old=json.loads((OUT.parent/'left_arm_general_model/v18_camera_board_registration.json').read_text())
K=np.array(old['camera_matrix'])
corners=np.load(OUT/'grid.npy').reshape(7,7,2)
if corners[-1,:,1].mean()<corners[0,:,1].mean():corners=corners[::-1]
if corners[:,-1,0].mean()<corners[:,0,0].mean():corners=corners[:,::-1]
pixels=corners.reshape(-1,2).astype(float)
obj=np.array([[x,y,0.] for y in range(385,0,-55) for x in range(55,386,55)],float)
ok,rv,tv=cv2.solvePnP(obj,pixels,K,None);assert ok
R=cv2.Rodrigues(rv)[0];uv=cv2.projectPoints(obj,rv,tv,K,None)[0][:,0]
vanish=K@R[:,2];vanish/=vanish[2]
result={'status':'independent camera/board diagnostic only; not deployed','camera_matrix':K.tolist(),'board_to_camera_rotation':R.tolist(),'board_to_camera_translation_mm':tv.ravel().tolist(),'vertical_vanishing_point_px':vanish[:2].tolist(),'reprojection_error_px':{'median':float(np.median(np.linalg.norm(uv-pixels,axis=1))),'max':float(np.max(np.linalg.norm(uv-pixels,axis=1)))},'normal_change_from_oct27_deg':float(np.degrees(np.arccos(np.clip(R[:,2]@np.array(old['board_to_camera_rotation'])[:,2],-1,1)))),'assumptions':['55 mm square size from existing board config','unchanged camera intrinsics; zero distortion as previous registration']}
(OUT/'camera_board_registration.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
