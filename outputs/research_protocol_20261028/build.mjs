import fs from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import {Workbook, SpreadsheetFile} from '@oai/artifact-tool';
const out=fileURLToPath(new URL('.',import.meta.url));
const wb=Workbook.create();
const materials=[];
const add=(category,item,qty,spec,status,condition,source)=>materials.push([`M${String(materials.length+1).padStart(2,'0')}`,category,'• '+item,qty,spec,status,condition,source]);
const actual='Current code / records'; const confirm='Confirm before use'; const required='Protocol requirement'; const reference='Reference design only';
add('Compute','NVIDIA Jetson Orin computer',1,'Installed Orin family; exact module, RAM and carrier SKU unknown.',actual,'Record module and carrier labels; verify rated input voltage.','Project logs; scripts/left_arm_v2_6.py');
add('Compute','Fanless ML computer / cooling solution',null,'Fan-free operation requested; no installed fanless computer verified.',confirm,'Use a manufacturer-rated fanless system if required; do not remove a required cooling fan.','User requirement; hardware label needed');
add('Compute','Jetson storage',null,'Boot medium, capacity and free space not yet inventoried.',confirm,'Record medium and available GB; retain sufficient space for planned logs/video.','Jetson inventory needed');
add('Compute','Development computer',1,'Windows PC with VS Code and SSH; processor/RAM unspecified.',actual,'Record OS, RAM and disk space for reproducibility.','Project workspace / SSH workflow');
add('Compute','Network router / Ethernet or Wi-Fi link',1,'Local connection between PC, Web interface and Jetson.',actual,'Record connection type; loss of network must not be treated as an emergency stop.','Project Web / SSH records');
add('Control','Damiao USB-to-CAN adapter',1,'Exact commercial model and revision unconfirmed.',actual,'Record adapter label and CAN bitrate; prevent concurrent serial ownership.','Left-arm installation records; DM_CAN.py');
add('Control','USB data cable',1,'Jetson-to-CAN adapter; connector types require confirmation.',required,'Use a data-rated cable with strain relief; route outside joint travel.','Physical inventory needed');
add('Control','Motor controller electronics',8,'Controllers integrated with the eight Damiao motors.',actual,'No separate Arduino controller is evidenced in the current control chain.','scripts/left_arm_v2_6.py; DM_CAN.py');
add('Control','Separate power-distribution / controller board',null,'Installed board manufacturer and revision unknown.',confirm,'Record board input/output ratings and wiring before powering.','Physical inventory needed');
add('Reference','RBE_Board V2.0',null,'Reference: 48 V XT60 input; 5 V USB-C output up to 8 A; four 48 V XT30 outputs.',reference,'Not confirmed installed; its 5 V output is not proof of compatibility with this Jetson carrier.','V2.0/roboto_origin_pcb/README.md');
add('Reference','Roboto_Usb2Can V1.0',null,'Reference: USB-C 5 V input; four CAN channels on GH1.25.',reference,'Do not substitute this reference board for the confirmed Damiao adapter without verification.','V1.0/atom01_pcb/Roboto_Usb2Can/README.md');
for(const [joint,model,id] of [['shoulder_front','DM4340','0x1E'],['shoulder_side','DM4340','0x1F'],['shoulder_rotate','DM4310','0x20'],['elbow','DM4340','0x21'],['arm_roll','DM4310','0x22'],['wrist_side','DM4310','0x2A'],['wrist','DM4310','0x29'],['claw','DM4310','0x28']]) add('Actuation',`${joint} motor`,1,`${model}; configured slave ID ${id}.`,actual,'Verify label/ID before replacement; seven arm axes plus one claw actuator.','scripts/left_arm_v2_6.py — JOINTS');
add('Wiring','CAN twisted-pair harness',null,'Length, gauge, connector count and topology require an as-built inventory.',required,'Check pinout, continuity and routing with power isolated.','DM_CAN.py; physical inventory');
add('Wiring','CAN termination components',null,'Count and resistance per adapter/motor wiring documentation.',confirm,'Confirm documented termination; do not assume every motor needs a terminator.','Adapter and motor manuals needed');
add('Wiring','Motor-power harness',null,'Gauge, current rating, polarity and connector types unverified.',confirm,'Match to confirmed supply and branch current ratings.','As-built wiring / labels needed');
add('Wiring','Cable sleeves, clamps and strain relief',null,'Sizes and quantities from actual harness routing.',required,'No taut cable or pinch point throughout approved travel.','Physical inspection');
add('Wiring','Insulated connector covers and terminal labels',null,'Match installed connectors; uniquely label motor branches.',required,'No exposed energized conductor accessible during operation.','Proposed electrical checklist');
add('Power','Battery pack, if battery powered',null,'Chemistry, nominal/max voltage, Ah, discharge limit and BMS unknown.',confirm,'Do not infer arm ratings from the older 3S/4S shopping proposal.','Battery nameplate required');
add('Power','Bench supply, if externally powered',null,'Installed supply model, voltage and current limit unverified.',confirm,'Use only ratings approved for the actual motors, board and wiring.','Supply nameplate required');
add('Power','Battery charger, if applicable',null,'Must match confirmed chemistry, series count and charge rating.',confirm,'Use the specified charger in a designated charging area.','Battery / charger manuals needed');
add('Power','Jetson power supply / DC converter',1,'Input and output ratings depend on the installed carrier.',confirm,'Record polarity, voltage and current ratings; verify compatibility before power-on.','Carrier and converter labels needed');
add('Power','Branch fuses / overcurrent protection',null,'Type, DC voltage rating and current rating require wiring review.',required,'Qualified reviewer selects ratings for conductors and loads.','Electrical review required');
add('Power','Accessible DC isolation switch',1,'Rated for the actual DC system; exact hardware unverified.',required,'Isolation can remove torque; support the arm before intentional isolation.','Proposed safety equipment');
add('Structure','Approved revised arm STEP assembly',1,'RB000152_02_手臂完整(2).step; orientation approved against the real arm.',actual,'Use the approved revision; retain the original file.','User-approved CAD revision');
add('Structure','Arm link 1 — RB000152_02',null,'Reference CNC-machined 6061 aluminum; as-built process unconfirmed.',reference,'Derive actual left-arm count from revised STEP; no metal printing confirmed.','A2_CNCBOM.xlsx');
add('Structure','Arm link 2 — RB000153_02',null,'Reference CNC-machined 6061 aluminum.',reference,'Confirm actual part material and quantity.','A2_CNCBOM.xlsx');
add('Structure','Arm link 3 — RB000154_02',null,'Reference CNC-machined 6061 aluminum.',reference,'Confirm actual part material and quantity.','A2_CNCBOM.xlsx');
add('Structure','Arm link 4 — RB000155_01',null,'Reference CNC-machined 6061 aluminum.',reference,'Confirm actual part material and quantity.','A2_CNCBOM.xlsx');
add('Structure','Arm link 5 — RB000179_01',null,'Reference CNC-machined 6061 aluminum.',reference,'Confirm actual part material and quantity.','A2_CNCBOM.xlsx');
add('Structure','Shoulder bracket — RB000160_02',null,'Reference CNC-machined 6061 aluminum.',reference,'Confirm current assembly and mounting orientation.','A2_CNCBOM.xlsx; revised STEP');
add('Structure','Universal / flange connectors',null,'Reference RB000167_03 and RB000168_02: 6061; RB000166_02: steel.',reference,'List only connectors actually used in the revised assembly.','A2_CNCBOM.xlsx');
add('Structure','Printed torso / support parts',null,'PA6-CF body recorded; grades and individual part assignments unknown.',actual,'Inspect for cracks, loose joints and load-dependent deformation.','左臂装机测试记录_20260703.md');
add('Structure','Custom wrist-side / wrist brackets',null,'Revised seven-axis assembly; actual material and fabrication process unknown.',confirm,'Record drawing ID, material and fastener specification for each bracket.','Revised STEP; physical inventory');
add('Structure','Gripper palm and finger components',1,'Two-finger gripper; material/process not verified.',actual,'Inspect pinch points and finger attachment; do not assume PLA or metal printing.','Approved photos / revised STEP');
add('Structure','Aluminum-extrusion support stand',1,'Cross-section, alloy, cut lengths and joint hardware unverified.',actual,'Record actual profile and cut list; 4040 was a proposal, not an as-built confirmation.','Photos; 桌面抓取与底盘设计记录_20260707.md');
add('Structure','Stand corner brackets / base anchors',null,'Match actual extrusion slots and mounting surface.',required,'Verify stand fixation against the approved experiment load.','As-built stand inspection');
add('Structure','Fasteners, washers and locking hardware',null,'Thread sizes, grades, quantities and tightening torques not inventoried.',confirm,'Use drawing/manufacturer torques; do not invent tightening values.','Revised STEP / assembly drawings needed');
add('Structure','Mechanical arm support / cradle',1,'Independent support suitable for the arm and payload.',required,'Must support the load before isolation, reset or maintenance.','Prior loss-of-torque incident; HANDOFF_20261027.md');
add('Manufacture','External CNC / printing service',null,'Use only for replacement or reproduction; process selected per approved drawing.',required,'Metal printing is not established as the current process; specialist shop handles machining/powders.','A2_CNCBOM.xlsx; user fabrication question');
add('Vision','Nuwa HP60C RGB-D camera',1,'HP60C model confirmed; serial and calibration revision to record.',actual,'Rigidly mount outside collision envelope.','Nuwa-HP60C-Depth-Camera-main; project records');
add('Vision','Camera cable and rigid mount',1,'Connector type, mounting fasteners and cable length unverified.',required,'No camera movement between calibration and trials.','Camera / bench inspection');
add('Vision','Stable room / task lighting',1,'Record lighting setting; avoid glare on gripper and chessboard.',required,'Keep conditions fixed during repeated visual measurements.','Proposed measurement method');
add('Task','Chessboard and fixed table',1,'Record board dimensions and coordinate origin from current board model.',actual,'Secure board against movement; clear the tested path.','data/left_arm_board_grasp.json');
add('Task','White knight test piece',1,'Current target piece; record piece height and identification.',actual,'Keep absent for initial empty approach; do not close claw while grasp release is blocked.','Board-grasp trial records');
add('Metrology','Millimeter ruler / scale',1,'Readable scale positioned on the board plane.',required,'Record resolution; remove any loose scale from the collision path.','User observed drift; proposed measurement');
add('Metrology','Digital caliper',1,'For accessible unpowered parts only; resolution to record.',required,'Do not remeasure approved CAD geometry merely to replace it with an estimate.','Proposed inventory / fixture checks');
add('Metrology','Camera calibration target / visible fiducial',1,'Printed or rigid target, if available; dimensions verified before use.',required,'If unavailable, label video estimates qualitative; do not equate pixels to mm.','HANDOFF_20261027.md; visual calibration pending');
add('Tools','Correct-size insulated hand tools',1,'Hex keys, drivers and connector tools matched to fasteners.',required,'Use only after power isolation and load support.','Proposed assembly checklist');
add('Tools','Calibrated torque driver',1,'Range suitable for drawing-specified tightening torques.',required,'Record torque settings and calibration status.','Fastener / motor documentation needed');
add('Tools','Rated digital multimeter',1,'Suitable for confirmed DC voltage; intact insulated probes.',required,'Electrical checks by a competent person; no exposed live probing during robot motion.','Proposed electrical checklist');
add('Safety','Physical emergency-stop system',1,'Installed hardware, behavior and restart method require verification.',required,'Do not treat a Web button as a safety-rated physical stop; support before stop verification.','Prior stop behavior; safety review required');
add('Safety','Barrier / marked exclusion zone',1,'Covers full approved swept volume and reviewer-approved clearance.',required,'Zero hands, people or loose objects inside during motion.','Proposed bench protocol');
add('Safety','Safety glasses / goggles',1,'Impact-rated eye protection suitable for the local work.',required,'Wear during setup and testing; prescription glasses alone may be inadequate.','Proposed PPE; local lab rules');
add('Safety','Cut-resistant handling gloves',1,'For sharp metal or printed-part handling with power isolated.',required,'Remove before work near moving joints, gripper or rotating components.','Proposed PPE; entanglement avoidance');
add('Safety','Closed-toe shoes and fitted clothing',1,'Tie back hair; remove dangling jewelry.',required,'No loose items near motor shafts or gripper linkages.','Proposed PPE / bench rules');
add('Safety','Battery incident provisions',1,'Facility-approved response plan and equipment for actual battery chemistry.',required,'Confirm charging location, exit route and response with the lab safety lead.','Battery chemistry not yet verified');
add('Safety','Warning sign and operator / observer checklist',1,'Named operator and observer; visible motion warning.',required,'Observer remains outside swept volume with stop access.','Proposed bench protocol');
add('Software','Jetson OS and device drivers',1,'Record Ubuntu / JetPack / driver versions; versions not pinned here.',actual,'Archive versions with each experiment batch.','Jetson environment inventory');
add('Software','Python 3 environment',1,'NumPy, PySerial and project dependencies; record exact versions.',actual,'Use a reproducible environment; one process owns the CAN serial device.','scripts/*.py; DM_CAN.py');
add('Software','Motor transport library',1,'DM_CAN.py; MIT gain encoding Kp 0–500 and Kd 0–5 in inspected driver.',actual,'Use per-joint configured limits; encoding range is not an experimentally safe gain range.','Inspected DM_CAN.py; project logs');
add('Software','Legacy motion / Web application',1,'left_arm_v2_6.py; left_arm_v2_8.py; existing Web entry point.',actual,'Retain ordinary Clearance shoulder_rotate exclusion.','scripts; HANDOFF_20261027.md');
add('Software','General dynamics and board-grasp control',1,'left_arm_general_motion.py; left_arm_board_grasp.py.',actual,'Use approved revision; no offset compensation or recorded-angle replay substituted for IK.','scripts; HANDOFF_20261027.md');
add('Software','Model Hold and adaptive learning',1,'left_arm_v3_1_model_control.py and current learning files.',actual,'Retain same-process / same-serial takeover and current adaptive state.','scripts; HANDOFF_20261027.md');
add('Software','Model and calibration JSON files',3,'left_arm_general_dynamics.json; left_arm_v3_1_dynamics.json; left_arm_board_grasp.json.',actual,'Back up actual revisions; record hashes before testing.','scripts/data');
add('Software','Vision software',1,'HP60C driver / ROS tools and OpenCV; record versions.',actual,'Recorded audit RGB is 640×480 at 2 fps; not a camera native-rate specification.','HP60C files; trial video logs');
add('Software','ML training / inference stack',null,'PyTorch / Ultralytics YOLO and dataset, if used for piece recognition.',confirm,'Record actual package versions, weights, dataset split and workload; not required for empty motion tests.','datasets; chessboard_yolo work logs');
add('Software','Offline parameter fitting tools',1,'SciPy / NumPy where used; versions to capture.',actual,'Existing effective inertia fits are not independently measured link mass properties.','General model identification records');
add('Software','VS Code, Git and SSH / SCP',1,'Development, revision tracking and controlled file transfer.',actual,'Verify synchronized file hashes before loading a new build.','GITHUB_SYNC_20261028.md');
add('Software','Excel-compatible workbook viewer',1,'Microsoft Excel or equivalent supporting formulas.',required,'Retain blank unknown fields until confirmed; do not replace them with zeros.','This protocol workbook');
add('Records','Log / video storage and backups',1,'JSONL, full command logs, raw video, operator notes and configuration snapshots.',required,'Use unique trial IDs and timestamps; retain failed trials.','scripts/data; HANDOFF_20261027.md');

const steps=[];const step=(phase,action,criterion,record,release='Before / during approved testing')=>steps.push([steps.length+1,phase,action,criterion,record,release]);
step('Scope','Define the research question.','Evaluate model-based board positioning and motion-to-hold transition; no claim of successful universal grasping.','Written objective');
step('Scope','Set the lateral-error acceptance threshold.','Enter an approved value in Trial log B3; a blank means no pass/fail conclusion.','Threshold in mm','Research criterion; not a safety limit');
step('Scope','Set the height-error acceptance threshold.','Enter an approved value in Trial log E3; a blank means no pass/fail conclusion.','Threshold in mm','Research criterion; not a safety limit');
step('Scope','Choose the trial count.','Proposed design: 3 repeats per approved target and configuration.','Trial plan','Proposal; approve before testing');
step('Safety setup','Assign the experiment operator.','Exactly one person issues motion commands.','Operator name');
step('Safety setup','Assign the stop observer.','Observer has a clear view and access to the approved stop control.','Observer name');
step('Safety setup','Obtain the bench safety review.','Reviewer resolves power ratings, swept-volume clearance and loaded-stop behavior.','Approval and date','Release gate: no power until resolved');
step('Safety setup','Put on eye protection.','Suitable impact-rated glasses or goggles in place.','PPE check');
step('Safety setup','Secure loose clothing and hair.','Zero dangling items near joints or gripper.','PPE check');
step('Safety setup','Put on closed-toe shoes.','Both feet fully covered.','PPE check');
step('Unpowered inspection','Place the arm on its mechanical support.','Support bears the load without relying on motor torque.','Support check','Required before isolation / maintenance');
step('Unpowered inspection','Isolate the motor power.','Follow approved isolation procedure; arm remains mechanically supported.','Isolation check');
step('Unpowered inspection','Put on handling gloves if sharp parts require them.','Gloves used only while power is isolated.','Handling condition');
step('Unpowered inspection','Inspect the stand fixation.','No visibly loose anchor, extrusion joint or cracked mount.','Inspection findings');
step('Unpowered inspection','Inspect the printed structural parts.','No crack, delamination or visibly damaged load-bearing feature.','Part IDs / findings');
step('Unpowered inspection','Inspect the gripper attachments.','No loose palm, finger pivot or mounting fastener.','Inspection findings');
step('Unpowered inspection','Verify the assembly fastener torques.','Use drawing/manufacturer values with a suitable torque driver.','Specified / measured torque','Gate: missing torque data requires review');
step('Unpowered inspection','Inspect the cable routing.','No exposed conductor, taut cable or predicted pinch point.','Harness findings');
step('Unpowered inspection','Remove handling gloves.','No gloves near moving mechanisms.','PPE check');
step('Electrical inventory','Record the Jetson module and carrier labels.','Exact SKU, input rating and cooling configuration recorded.','Hardware inventory');
step('Electrical inventory','Record the USB-to-CAN adapter label.','Manufacturer, model and revision recorded.','Hardware inventory');
step('Electrical inventory','Record the power-source nameplate.','Voltage, polarity and current rating; battery chemistry/Ah/BMS if applicable.','Power inventory');
step('Electrical inventory','Verify power compatibility.','All supply, converter, motor, board, fuse and cable ratings reviewed as compatible.','Electrical sign-off','Release gate: do not infer voltage from reference BOM');
step('Electrical inventory','Verify the isolated CAN wiring.','Pinout and termination meet the actual device documentation.','Continuity / termination record');
step('Safety validation','Verify the approved stop behavior on the supported arm.','Approved procedure confirms response and restart; no unsupported load-drop test.','Stop-test record','Competent supervisor; before loaded motion');
step('Workspace','Mark the exclusion zone.','Boundary encloses the approved swept volume and reviewed clearance.','Zone sketch');
step('Workspace','Clear the exclusion zone.','Zero people, hands and loose objects inside during motion.','Clearance check');
step('Workspace','Fix the chessboard to the table.','No board movement relative to its registration fixture.','Fixture record');
step('Workspace','Fix the camera mount.','Camera remains stationary throughout calibration and trials.','Mount record');
step('Workspace','Set the lighting.','Same lighting setting used for all repeats.','Lighting setting');
step('Software preparation','Archive the synchronized code revision.','Record Git revision and synchronization verification.','Revision / hash');
step('Software preparation','Back up the model and adaptive-learning files.','Timestamped copy exists before trials.','Backup location');
step('Software preparation','Record the software environment.','Python, JetPack/OS, driver, NumPy, PySerial and vision versions captured.','Version inventory');
step('Software preparation','Verify exclusive serial ownership.','Exactly one authorized control process owns the configured adapter.','Process / device record');
step('Software preparation','Verify the preserved control settings.','Model Hold, adaptive learning and normal Clearance exclusion retained; shoulder_rotate Kp cap 36 / Kd 4.5.','Configuration snapshot');
step('Software preparation','Record the protection configuration.','Velocity/error watchdogs currently record-only; do not count them as automatic stopping safeguards.','Protection-mode snapshot');
step('Software preparation','Verify the permitted experiment stages.','Only approved above/touch/approach checks; grasp/closure remains blocked pending release.','Stage-release record');
step('Software preparation','Load the approved CAD and calibration revision.','Revised STEP and seven-axis calibration correspond to the current hardware.','CAD/model hash');
step('Thermal check','Verify cooling suitability for the intended workload.','Manufacturer limits available; fanless operation only if the hardware is designed for it.','Cooling review','Gate: no fan removal to meet fan-free request');
step('Thermal check','Run the approved ML workload without robot motion.','Use intended workload duration; arm stays supported and no motion command is issued.','Workload / duration','Only if ML is part of the experiment');
step('Thermal check','Record the compute thermal results.','Temperatures below manufacturer limits; record thermal-throttling events and inference rate.','Temperature / events / fps','Investigate any thermal throttling before trials');
step('Measurement setup','Record the board coordinate convention.','Document origin, X/Y directions, units and board normal from registration.','Coordinate diagram');
step('Measurement setup','Verify the visual measurement scale.','Use a verified target or board-plane scale; state resolution and uncertainty.','Scale calibration','Uncalibrated pixels are qualitative only');
step('Measurement setup','Choose an approved empty approach target.','Begin with an established target; arbitrary points require preview and workspace review.','Target X/Y/height');
step('Measurement setup','Remove the chess piece from the initial test path.','Zero pieces or obstacles in the planned empty trajectory.','Empty-path check');
step('Power and baseline','Restore power under the approved startup procedure.','Mechanical support remains available; observer ready.','Startup result','Only after all release gates pass');
step('Power and baseline','Read the stationary seven-axis status.','Log all seven measured positions, velocities and torques.','Baseline status');
step('Power and baseline','Run the approved Home workflow.','Observer confirms no contact, abnormal sound or visible oscillation.','Home log','Do not bypass a failed Home check');
step('Power and baseline','Verify the Home hold result.','Record completion and measured drift; do not infer accuracy from green LEDs.','Hold log / observation');
step('Planning','Enter the Cartesian target.','Use registered board mm coordinates; historical comparison: X113.8 / Y190.3 / height50.2, White Knight.','Target inputs','Historical target is not approval for a new run');
step('Planning','Preview the model-based trajectory.','Planner reports a valid path inside the approved operating envelope.','Preview / path ID','Known operating envelope is not a complete collision model');
step('Planning','Review the full path clearance.','No predicted contact with board, stand, camera, cables or pieces.','Operator / reviewer check');
step('Planning','Create a unique trial identifier.','One ID links target, configuration, video and logs.','Trial log ID');
step('Trial execution','Start the fixed-camera recording.','Recording includes the full approach, descent and hold.','Video file / timestamp');
step('Trial execution','Execute the approved Above Only stage.','One command; observer remains outside the swept volume.','Command / motion log');
step('Trial execution','Assess the above-target position.','Independent visual check distinguishes lateral, height and orientation errors.','Observed position','No further descent after unexpected contact or instability');
step('Trial execution','Preview the approved Touch Position descent.','Use current measured start pose; record requested descent and path.','Descent preview','No recorded-angle replay or hidden offset substitution');
step('Trial execution','Execute the approved Touch Position descent.','One descent command; claw remains open.','Command / trajectory log');
step('Trial execution','Record the physical descent behavior.','Separate movement during descent from movement after takeover; report B/D direction and mm.','Physical observation');
step('Trial execution','Verify the automatic Model Hold transition.','Same-process / same-serial takeover completes; record the configured 5 s hold result.','Takeover and hold log');
step('Trial execution','Record the final gripper location.','Measure X/Y/height independently; include scale uncertainty.','Trial log measurements');
step('Trial execution','Record the motion diagnostics.','Requested 0.01 s control interval and actual frame timing reported separately.','Timing / torque / joint-error summaries');
step('Trial execution','Record any endpoint-settle action.','Distinguish enabled, skipped and executed; note any final XYZ movement.','Settle count / observed movement');
step('Trial execution','Stop the recording.','Video includes the completed hold period.','Video end time');
step('Review','Compare the three lateral-axis plans with feedback.','Examine shoulder_side, shoulder_rotate and wrist_side during the descent window.','Planned / actual joint traces');
step('Review','Compare the encoder prediction with physical displacement.','Report both; encoder agreement alone does not prove a vertical path.','Model vs independent measurement');
step('Review','Evaluate the trial against the approved research thresholds.','Trial log remains Not evaluated when tolerances or measurements are absent.','Calculated result');
step('Review','Review any anomaly before another trial.','Unexpected contact, instability or fault requires the approved stop/recovery procedure.','Anomaly / recovery record','Do not disable a load-bearing arm without support');
step('Repeat trials','Return through the approved Home workflow.','Return completed before the next full-path repeat.','Home return log');
step('Repeat trials','Repeat the approved trial sequence.','Use the chosen repeat count; preserve all failed and successful records.','Repeat number','Only after previous trial review');
step('Grasp release','Obtain a separate grasp-stage release.','Closure and lifting stay blocked until physical positioning and load/stop conditions are approved.','Signed release','Current release does not permit grasp execution');
step('Grasp validation','Place the test piece at the approved target.','Robot stationary; access only under approved safe setup state.','Piece ID / position','Only after grasp-stage release');
step('Grasp validation','Execute the separately approved grasp routine.','Use released closure and lift parameters; never improvise grip force or lift height.','Released routine / result','Conditional future phase; not currently executable');
step('Shutdown','Return the arm to the approved parking pose.','Complete only through the reviewed motion workflow.','Parking log');
step('Shutdown','Place the arm on its mechanical support.','Support carries the load before torque removal.','Support check');
step('Shutdown','Isolate motor power.','Follow approved shutdown sequence; no unsupported drop.','Shutdown check');
step('Shutdown','Archive the complete trial records.','Code/model hashes, adaptive state, raw video, logs and observations retained together.','Archive / backup');
step('Shutdown','Inspect the fixture for post-test damage.','Record any loose fastener, crack or cable wear with power isolated.','Post-test inspection');

function base(name,headers,rows,widths,notes){
 const s=wb.worksheets.add(name);s.showGridLines=false;
 s.getRangeByIndexes(0,0,rows.length+6,headers.length).format.font={name:'Arial',size:10};
 s.getRange('A1').values=[[name==='Materials'?'HiveRobot | Materials and equipment':name==='Procedure'?'HiveRobot | Numbered experiment procedure':'HiveRobot | Trial measurements']];
 s.getRange('A1').format.font={name:'Arial',size:16,bold:true,color:'#15364C'};
 s.getRange('A1:H1').format.rowHeight=30;
 notes.forEach((n,i)=>{s.getCell(i+1,0).values=[[n]];s.getRangeByIndexes(i+1,0,1,headers.length).format.rowHeight=24;});
 s.getRangeByIndexes(4,0,1,headers.length).values=[headers];
 if(rows.length)s.getRangeByIndexes(5,0,rows.length,headers.length).values=rows;
 const range=s.getRangeByIndexes(4,0,rows.length+1,headers.length);range.format.wrapText=true;range.format.verticalAlignment='center';
 range.format.rowHeight=62;
 s.getRangeByIndexes(4,0,1,headers.length).format.rowHeight=30;
 const t=s.tables.add(range.address??`${'A'}5:${String.fromCharCode(64+headers.length)}${rows.length+5}`,true,name.replace(/ /g,'')+'Table');
 t.showFilterButton=true;
 for(let i=0;i<headers.length;i++)s.getRangeByIndexes(0,i,rows.length+6,1).format.columnWidth=widths[i];
 s.getRangeByIndexes(4,0,1,headers.length).format.fill='#15364C';s.getRangeByIndexes(4,0,1,headers.length).format.font={bold:true,color:'#FFFFFF',name:'Arial',size:10};
 for(let i=0;i<rows.length;i++)if(i%2===1)s.getRangeByIndexes(i+5,0,1,headers.length).format.fill='#F0F5F7';
 s.freezePanes.freezeRows(5);s.freezePanes.freezeColumns(name==='Procedure'?2:1);return s;
}
const shortMaterials=materials.filter(r=>r[1]!=='Reference'&&r[1]!=='Manufacture'&&!r[2].includes('Motor controller electronics')).map(r=>{
 let name=r[2];
 if(r[1]==='Actuation')name+=` (${r[4].split(';')[0]})`;
 if(name.includes('Camera cable and rigid mount'))return [name,'1 set'];
 if(name.includes('gloves'))return [name,'1 pair'];
 if(name.includes('Closed-toe shoes'))return [name,'1 set'];
 if(name.includes('Model and calibration JSON'))name='• Dynamics and board-calibration JSON files';
 return [name,r[3]??'TBC'];
});
const m=base('Materials',['Item','Quantity'],shortMaterials,[83,16],[
 'TBC = quantity not confirmed. Quantities refer to the experimental setup, not purchase totals.',
 'Exact battery, Jetson variant, board and structural material specifications remain to be confirmed.',
 'Arm-link part numbers follow the reference CAD; as-built quantities remain TBC.'
]);
const shortSteps=[
 'Assign one command operator.',
 'Assign one stop observer.',
 'Wear impact-rated eye protection.',
 'Secure loose clothing, jewelry and hair.',
 'Support the arm mechanically.',
 'Isolate motor power before inspection.',
 'Use handling gloves only for sharp, unpowered parts.',
 'Inspect the stand and arm for damage.',
 'Verify fastener tightness against specified torques.',
 'Inspect power and CAN wiring.',
 'Confirm power-source compatibility.',
 'Remove handling gloves before motion.',
 'Verify stop behavior with the arm mechanically supported.',
 'Mark the approved motion exclusion zone.',
 'Clear all people and loose objects from that zone.',
 'Secure the chessboard.',
 'Secure the camera.',
 'Back up code, models and adaptive-learning files.',
 'Confirm that only one process owns the motor adapter.',
 'Verify preserved Model Hold, learning and Clearance settings.',
 'Confirm cooling suitability for the intended ML workload.',
 'Record software and hardware versions.',
 'Set approved lateral and height tolerances in the trial log.',
 'Calibrate the board-plane measurement scale.',
 'Restore power using the approved startup procedure.',
 'Record stationary seven-axis feedback.',
 'Run the approved Home workflow.',
 'Enter the approved Cartesian target in millimeters.',
 'Preview the model-generated path.',
 'Verify clearance along the full path.',
 'Start the fixed-camera recording.',
 'Execute Above Only with an empty path.',
 'Check the physical above-target position.',
 'Preview Touch Position from the measured current pose.',
 'Execute the approved open-claw descent.',
 'Observe the 5-second automatic Model Hold.',
 'Record independent X, Y and height measurements.',
 'Record lateral drift during descent and after takeover.',
 'Save the joint-feedback and timing logs.',
 'Repeat the approved sequence three times per target.',
 'Keep claw closure blocked until grasp testing is released.',
 'Return through the approved Home workflow.',
 'Support the arm mechanically before shutdown.',
 'Isolate motor power.',
 'Archive the trial records.'
].map((a,i)=>[i+1,a]);
const p=base('Procedure',['Step','Action'],shortSteps,[9,101],[
 'Review draft: confirm electrical ratings, stop behavior and the operating envelope before testing.',
 'Unexpected contact or instability: follow the approved stop procedure; never isolate an unsupported load.',
 'Three repeats are proposed. Current software watchdogs record errors; grasp closure remains unreleased.'
]);
for(const s of [m,p]){
 s.getUsedRange().format.rowHeight=27;
 s.getRange('A1:B1').format.rowHeight=34;
 s.getRange('A2:B4').format.rowHeight=25;
 s.getRange('A5:B5').format.rowHeight=30;
 s.freezePanes.unfreeze();s.freezePanes.freezeRows(5);
}
shortMaterials.forEach((r,i)=>{if(r[1]==='TBC')m.getRange(`B${i+6}`).format.fill='#FFF0CE';});
const trialRows=Array.from({length:24},(_,i)=>[null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null,null]);
const tr=base('Trial log',['Trial ID','Target / stage','Repeat','Target X mm','Target Y mm','Target H mm','Measured X mm','Measured Y mm','Measured H mm','X error mm','Y error mm','H error mm','XY error mm','Evaluation','During descent / after hold','Safety / motion observation','Log / video reference','Config / operator'],trialRows,[18,24,9,13,13,13,15,15,15,13,13,13,14,20,36,37,35,28],[
 'Enter independent physical measurements, not FK outputs. Positive X direction must follow your registered board convention.',
 'Lateral tolerance (mm)',
 'Height tolerance is in E3. Both tolerances are blank until approved; evaluation is a research check, not a safety certification.'
]);
tr.getRange('D3').values=[['H limit mm']];
tr.getRange('A3').values=[['Lateral tolerance (mm)']];tr.getRange('B3').values=[[null]];tr.getRange('E3').values=[[null]];
tr.getRange('B3').format.fill='#FFF0CE';tr.getRange('E3').format.fill='#FFF0CE';
tr.getRange('B3').format.font={name:'Arial',size:10,color:'#1759A6'};tr.getRange('E3').format.font={name:'Arial',size:10,color:'#1759A6'};
tr.getRange('A4').values=[['Proposed: 3 repeats per released target. Historical 27 Oct trial: smooth motion, ~10 mm B drift observed; not entered as a new measurement.']];
tr.getRange('A2').values=[['Inputs: blue text. Calculated errors: black. Blank inputs remain unavailable; enter a true zero only when measured.']];
tr.getRange('A6:I29').format.font={name:'Arial',size:10,color:'#1759A6'};
tr.getRange('O6:R29').format.font={name:'Arial',size:10,color:'#1759A6'};
tr.getRange('D6:M29').setNumberFormat('0.0');
for(let row=6;row<=29;row++){
 for(const [col,meas,target] of [['J','G','D'],['K','H','E'],['L','I','F']])tr.getRange(`${col}${row}`).formulas=[[`=IF(AND(ISNUMBER(${meas}${row}),ISNUMBER(${target}${row})),${meas}${row}-${target}${row},"")`]];
 tr.getRange(`M${row}`).formulas=[[`=IF(AND(ISNUMBER(J${row}),ISNUMBER(K${row})),SQRT(J${row}^2+K${row}^2),"")`]];
 tr.getRange(`N${row}`).formulas=[[`=IF(AND(ISNUMBER($B$3),ISNUMBER($E$3),$B$3>0,$E$3>0,ISNUMBER(M${row}),ISNUMBER(L${row})),IF(AND(M${row}<=$B$3,ABS(L${row})<=$E$3),"Within criterion","Outside criterion"),"Not evaluated")`]];
}
// Formula guards intentionally preserve missing measurements and missing acceptance limits.
wb.recalculate();
console.log('COUNTS',JSON.stringify({materials:materials.length,procedure:steps.length,trialSlots:24}));
console.log((await wb.inspect({kind:'region',sheetId:'Trial log',range:'J6:N7',maxChars:1800,tableMaxCols:5})).ndjson);
console.log((await wb.inspect({kind:'match',searchTerm:'#REF!|#DIV/0!|#VALUE!|#NAME\\?|#NUM!|#SPILL!|#CALC!',options:{useRegex:true,maxResults:20},maxChars:1800})).ndjson);
for(const [name,range] of [['Materials','A1:B20'],['Procedure','A1:B22'],['Trial log','A3:N9']]){
 const png=await wb.render({sheetName:name,range,scale:1.25,format:'png'});
 await fs.writeFile(`${out}/${name.replace(/ /g,'_')}_preview.png`,new Uint8Array(await png.arrayBuffer()));
}
const x=await SpreadsheetFile.exportXlsx(wb);await x.save(`${out}/HiveRobot_Research_Materials_and_Protocol.xlsx`);
await fs.writeFile(`${out}/content_audit.json`,JSON.stringify({materials,steps},null,2));
