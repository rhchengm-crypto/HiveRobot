from pathlib import Path
source=(Path(__file__).resolve().parent/'deploy_optical_board_candidate.py').read_text()
source=source.replace('optical_candidate_plan_verification.json','touch_candidate_plan_verification.json')
source=source.replace('left_arm_board_grasp_optical_candidate.json','left_arm_board_grasp_touch_candidate.json')
source=source.replace('board_model_before_optical.json','board_model_before_touch.json')
source=source.replace('optical_registration_deployment.json','touch_plane_deployment.json').replace('v23_optical_','v24_touch_')
source=source.replace('board registration only; no motor-controller or learning-file change','retain optical normal and XY; touch-referenced plane height; no motor-controller or learning-file change')
exec(compile(source,__file__,'exec'))
