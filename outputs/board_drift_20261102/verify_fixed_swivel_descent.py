"""Regression checks against real approach-entry feedback, no hardware imports."""
exec((__import__('pathlib').Path(__file__).resolve().parent/'evaluate_redundancy_constraint.py').read_text().split("for lock in")[0])
from left_arm_board_grasp import feedback_contact_path, check_transit_edge
from left_arm_board_kinematics import solve_ik
entry=next(json.loads(l.split('=',1)[1]) for l in lines if l.startswith('board feedback descent entry='))
measured=entry['start_measured_rad']
rows, diagnostic=feedback_contact_path(m,c,measured,[113.8,190.3,50.2])
contact=[r for r in rows if r['stage']=='contact']
assert contact and all(abs(r['encoder_rad']['shoulder_rotate']-diagnostic['fixed_descent_encoder_rad']['shoulder_rotate'])<1e-12 for r in contact)
assert max(r['position_error_mm'] for r in rows)<=.5
assert max(r['rotation_error_deg'] for r in rows)<=np.degrees(.005)
q=np.array([measured[n] for n in m.names]);peak_step=0.
for row in rows:
 nextq=np.array([row['encoder_rad'][n] for n in m.names]);check_transit_edge(m,q,nextq,c)
 peak_step=max(peak_step,float(np.max(np.abs(np.degrees(nextq-q)))));q=nextq
# A feedback deviation must not change the fixed reference used by the solver.
seed=q.copy();seed[m.names.index('shoulder_rotate')]+=.001
p=tcp_pose(m,q,tcp)
a=solve_ik(m,p[:3,3],seed,tcp,lim,p[:3,:3],fixed_encoder=diagnostic['fixed_descent_encoder_rad'])
assert a['success'] and abs(a['encoder_rad']['shoulder_rotate']-q[m.names.index('shoulder_rotate')])<1e-12
report={'passed':True,'waypoints':len(rows),'contact_waypoints':len(contact),'max_position_error_mm':max(r['position_error_mm'] for r in rows),'max_rotation_error_deg':max(r['rotation_error_deg'] for r in rows),'max_step_deg':peak_step,'fixed_encoder_rad':diagnostic['fixed_descent_encoder_rad'],'endpoint_encoder_rad':contact[-1]['encoder_rad'],'hardware_motion_issued':False}
(OUT/'fixed_swivel_verification.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
