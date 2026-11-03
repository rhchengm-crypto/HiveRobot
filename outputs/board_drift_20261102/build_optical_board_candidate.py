"""Use known CAD plate + fixed camera + live board corners; no motor commands."""
import json,subprocess,shlex,base64,sys,hashlib,copy
from pathlib import Path
import cv2,numpy as np
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
code=r'''
import rospy,cv2,numpy as np,base64
from sensor_msgs.msg import Image
from record_left_arm_rgbd_audit import image_array
rospy.init_node('hive_readonly_board_snapshot',anonymous=True,disable_signals=True)
m=rospy.wait_for_message('/ascamera_hp60c/rgb0/image',Image,timeout=5);im=image_array(m,np)
if m.encoding=='rgb8':im=im[:,:,::-1]
print(base64.b64encode(cv2.imencode('.png',im)[1].tobytes()).decode())
'''
shell='source /opt/ros/noetic/setup.bash && cd /home/nvidia/hive_robot/DM_Control_Python && exec python3 -c '+shlex.quote(code)
r=subprocess.run(['ssh','-C','-i',str(Path.home()/'.ssh/hiverobot_jetson_sync_ed25519'),'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','ConnectTimeout=5','nvidia@192.168.201.199','bash -c '+shlex.quote(shell)],capture_output=True,timeout=25)
if r.returncode:raise RuntimeError(r.stderr.decode(errors='replace'))
raw=base64.b64decode(r.stdout);(OUT/'board_before_optical_calibration.png').write_bytes(raw);im=cv2.imdecode(np.frombuffer(raw,np.uint8),1)
crop=im[25:380,275:590];found,corners=cv2.findChessboardCornersSB(crop,(4,7),flags=cv2.CALIB_CB_NORMALIZE_IMAGE|cv2.CALIB_CB_EXHAUSTIVE)
obj=np.array([[x,y,0.] for y in range(385,0,-55) for x in range(220,386,55)],float)
K=np.array(json.loads((OUT/'camera_board_registration.json').read_text())['camera_matrix'])
method='rectangular chessboard detector'
if found:
 pixels=corners.reshape(7,4,2)+[275,25]
 if pixels[-1,:,1].mean()<pixels[0,:,1].mean():pixels=pixels[::-1]
 if pixels[:,-1,0].mean()<pixels[:,0,0].mean():pixels=pixels[:,::-1]
else:
 # Reacquire already indexed board corners in a stationary scene, not unlabelled features.
 previous=json.loads((OUT/'restored_board_camera_registration.json').read_text())
 rv0=cv2.Rodrigues(np.array(previous['board_to_camera_rotation']))[0]
 expected=cv2.projectPoints(obj,rv0,np.array(previous['board_to_camera_translation_mm']),K,None)[0].astype(np.float32)
 gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
 points=cv2.cornerSubPix(gray,expected.copy(),(8,8),(-1,-1),(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,80,.001))
 assert np.max(np.linalg.norm(points-expected,axis=-1))<8,'Board moved beyond corner reacquisition window'
 for x,y in np.rint(points[:,0]).astype(int):
  a=float(gray[y-4,x-4]);b=float(gray[y-4,x+4]);cc=float(gray[y+4,x-4]);dd=float(gray[y+4,x+4])
  assert abs(a+dd-b-cc)>40,'Corner failed checker contrast check'
 pixels=points.reshape(7,4,2);method='prior-indexed corners reacquired with subpixel refinement and checker contrast checks'
ok,rv,tv=cv2.solvePnP(obj,pixels.reshape(-1,2),K,None);assert ok
Rb=cv2.Rodrigues(rv)[0];tb=tv.ravel()/1000;project=cv2.projectPoints(obj,rv,tv,K,None)[0][:,0];error=np.linalg.norm(project-pixels.reshape(-1,2),axis=1);assert max(error)<1
fit=json.loads((OUT/'known_plate_calibration_candidates.json').read_text())[0];assert not fit['active_zero_joints']
assert fit['independent_v22']['median_px']<1 and fit['independent_v22']['p95_px']<2
Rc=np.array(fit['camera_rotation']);tc=np.array(fit['camera_translation_m']);R=Rc.T@Rb;t=Rc.T@(tb-tc)
c=json.loads((ROOT/'scripts/data/left_arm_board_grasp.json').read_text());oldR=np.array(c['board_to_cad_rotation']);oldt=np.array(c['board_to_cad_translation_m'])
candidate=copy.deepcopy(c);candidate['board_to_cad_rotation']=R.tolist();candidate['board_to_cad_translation_m']=t.tolist()
candidate['build']='seven-axis-board-grasp-v23-optical-registration'
candidate['historical_teaching_fit_errors_mm']=candidate['fit_errors_mm']
candidate['fit_errors_mm']={}
# Re-express already taught orientation data in the corrected board frame.
for value in list(candidate['profiles'].values())+candidate.get('orientation_hints',[]):
 if 'wrist_rotation_in_board' in value:value['wrist_rotation_in_board']=(R.T@oldR@np.array(value['wrist_rotation_in_board'])).tolist()
candidate['registration_status']='fixed-camera optical registration from CAD plate and board grid; pending on-robot Cartesian validation'
candidate['independently_validated']=False
candidate['optical_registration']={'source':'six CAD holes on existing wrist plate + board intersections','source_step_sha256':c['source_step_sha256'],'robot_joint_zero_changes':False,'camera_fit_training_run':'arm_run_1793671770732_5069.txt','independent_validation_run':'arm_run_1793679218917_10319.txt','camera_fit_validation':fit['independent_v22'],'live_board_reprojection_max_px':float(max(error)),'calibration_scope':'board-to-CAD transform; no fixed Cartesian error offset','old_transform':{'rotation':oldR.tolist(),'translation_m':oldt.tolist()},'physical_grasp_validation_complete':False}
(OUT/'left_arm_board_grasp_optical_candidate.json').write_text(json.dumps(candidate,indent=2))
sys.path.insert(0,str(ROOT/'scripts'));from left_arm_rigid_body import RigidBodyModel
from left_arm_board_kinematics import tcp_pose
m=RigidBodyModel.load(ROOT/'scripts/data/left_arm_general_dynamics.json');predictions=[]
for log in ['arm_run_1793671770732_5069.txt','arm_run_1793679218917_10319.txt']:
 lines=(OUT/'jetson_snapshot'/log).read_text().splitlines();waypoint=next(json.loads(l.split('=',1)[1]) for l in lines if l.startswith('board waypoint=') and json.loads(l.split('=',1)[1])['stage']=='contact');aud=[json.loads(l.split('=',1)[1]) for l in lines if l.startswith('board tracking audit=') and json.loads(l.split('=',1)[1])['stage']=='contact']
 qa=np.array([waypoint['entry_feedback'][n]['pos'] for n in m.names]);qb=np.array([aud[-1]['feedback'][n]['pos'] for n in m.names]);delta=tcp_pose(m,qb,c['tcp_in_wrist_frame_m'])[:3,3]-tcp_pose(m,qa,c['tcp_in_wrist_frame_m'])[:3,3]
 predictions.append({'log':log,'old_board_model_delta_mm':(oldR.T@delta*1000).tolist(),'optically_registered_board_delta_mm':(R.T@delta*1000).tolist()})
report={'board_corner_method':method,'board_reprojection_median_px':float(np.median(error)),'board_reprojection_max_px':float(max(error)),'old_board_up_in_optical_board':(R.T@oldR[:,2]).tolist(),'old_trajectories_recomputed':predictions,'candidate':'left_arm_board_grasp_optical_candidate.json','deployed':False,'motion_issued':False}
(OUT/'optical_board_candidate_audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
