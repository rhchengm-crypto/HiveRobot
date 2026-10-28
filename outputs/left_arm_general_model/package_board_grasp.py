"""Package only new board integration files; never copy Jetson learning data."""
import hashlib
import json
from pathlib import Path
import zipfile

root=Path(__file__).resolve().parents[2]
output=Path(__file__).resolve().parent
relative=['left_arm_board_grasp.py','left_arm_board_kinematics.py',
          'left_arm_fault_support.py','left_arm_board_home_handoff.py','left_arm_v3_1_model_control.py',
          'left_arm_general_motion.py','left_arm_dynamics_worker.py','left_arm_v3_1_motion_dynamics.py',
          'chessboard_vision_v3_0_web_control.py','data/left_arm_board_grasp.json',
          'LEFT_ARM_BOARD_GRASP_DEPLOY.md']
manifest={'build':'seven-axis-board-grasp-v9-stiffness-handoff','date':'2026-10-27',
          'status':'retained-motion-gains-for-fixed-pose-hold; hardware-retest-pending',
          'released_execution_stages':['above','touch'],
          'automatic_endpoint_correction':False,
          'physical_joint_limit_parameters_available':False,
          'offline_workspace_successes':33,'offline_workspace_tested':64,
          'focused_regression_tests_passed':130,
          'home_180_regression_tests_passed':68,
          'fault_support_tests_passed':16,
          'fault_support_hardware_validated':False,
          'files':{name:hashlib.sha256((root/'scripts'/name).read_bytes()).hexdigest() for name in relative},
          'preserved':['existing Model Hold bridge','existing adaptive learning and files',
                       'ordinary Clearance shoulder_rotate active-target skip'],
          'old_learning_files_in_package':False}
(output/'board_grasp_sync_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
for filename in ('left_arm_board_grasp_update.zip','left_arm_board_grasp_v9_update.zip'):
    with zipfile.ZipFile(output/filename,'w',zipfile.ZIP_DEFLATED) as archive:
        for name in relative:
            archive.write(root/'scripts'/name,'DM_Control_Python/'+name)
        archive.write(output/'board_grasp_sync_manifest.json','DM_Control_Python/board_grasp_sync_manifest.json')
progress=output/'robot_validation_progress.json'
data=json.loads(progress.read_text(encoding='utf8'))
previous=data.get('board_grasp_integration',{})
previous.update({k:v for k,v in manifest.items() if k!='files'})
previous.update(execution_suspended=False, fault_stop_software_repaired=True, velocity_and_tracking_stop_enabled=False,
                fault_stop_repair_validated=False, on_robot_test_passed=False,
                loaded_stop_observed=True,
                loaded_stop_log='arm_run_1793140179675_43265.txt',
                next_test='Home then Above Only X132.5 Y192.5; verify no extra final motion and C4 alignment before descent')
data['board_grasp_integration']=previous
progress.write_text(json.dumps(data,indent=2),encoding='utf8')
print(json.dumps(manifest,indent=2))
