"""Record software completion separately from the operator's failed height."""
from pathlib import Path
import datetime,hashlib,json

ROOT=Path(__file__).resolve().parents[3]
OUT=Path(__file__).resolve().parents[1]
def read(name):return json.loads((OUT/name).read_text(encoding='utf8'))
audit=read('execution_audit.json');feedback=read('operator_feedback.json')
height=read('height_diagnosis.json');downloads=read('download_verification.json')
collection=read('collection_report.json');learning=read('learning_sync/old_new_hash_audit.json')
assert all(audit['checks'].values()) and not feedback['full_motion_accepted']
record=dict(schema=1,recorded_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    trial='a8_v33_execution_20261108',build=audit['build'],
    status='full empty-load motion completed; lateral drift resolved; gripping height failed',
    physical_validation_accepted=False,fitting_eligible=False,successful_path_library_updated=False,
    software_command_ok=audit['command_result_ok'],request=audit['request'],
    original_nominal_A8_XYZ_solved=False,
    request_scope='New geometric inverse path constrained by approved A8_take endpoint; no saved trajectory replay',
    phases='ordinary fixed Clearance -> full approach and descent -> 5s same-serial hold; no claw closure',
    home_executed_as_part_of_this_trial=False,antecedent_Home_confirmed_stable=True,
    operator_feedback=feedback,controller_checks=audit['checks'],
    motor_frames=audit['trace']['motor_frames'],confirmed_oscillation_events=audit['trace']['confirmed_oscillation_events'],
    fault_support_frames=audit['trace']['fault_support_frames'],
    baseline_observations_accepted=audit['v33_quasistatic_support_audit']['accepted_observations'],
    all_accepted_fresh_windows_reproduced=audit['v33_quasistatic_support_audit']['every_accepted_window_reproduced'],
    endpoint_max_joint_error_deg=audit['endpoint']['max_nominal_error_deg'],
    previous_v32_endpoint_max_joint_error_deg=2.5791187041845425,
    endpoint_joint_errors_deg=audit['endpoint']['nominal_error_deg'],
    same_serial_hold_seconds=audit['inline_hold']['duration_s'],
    source_files=audit['source_file_verification'],
    archives=downloads['archives'],RGBD_pairs=collection['camera']['paired_frames'],
    next_sync_expected_learning_sha256={r['name']:r['new_sha256'] for r in learning['rows']},
    height_diagnosis_path='outputs/a8_v33_execution_20261108/height_diagnosis.json',
    lower25_diagnostic_path='outputs/a8_v33_execution_20261108/lower25_endpoint_diagnostic.json',
    orientation_and_bound_diagnostics_path='outputs/a8_v33_execution_20261108/height_followup_feasibility.json',
    lower25_height_is_proposed_task_request_not_a_calibration_offset=True,
    additional_motion_after_operator_feedback=False,
    next_step='Continue offline common-model search for a lower viable task pose; no unverified path execution',
    evidence_directory='outputs/a8_v33_execution_20261108')
text=json.dumps(record,ensure_ascii=False,indent=2)+'\n'
(OUT/'trial_summary.json').write_text(text,encoding='utf8')
registry=ROOT/'scripts/data/board_workspace_trials/validation_20261108_a8_v33.json'
registry.write_text(text,encoding='utf8')
(OUT/'README.md').write_text('''# A8 v33 空载完整下降实测 · 2026-11-08

本轮已完成：普通固定 Clearance → 完整接近和下降 → 5 秒同串口 Model Hold，未闭爪。Home 为前一轮已完成的起点，本轮没有重复执行 Home。

**现场结论：横移问题解决；夹持高度仍偏高，目测至少 25 mm。整轮抓取验证未通过，不加入成功路径库，不作为完整位姿标定数据。** 用户未单独确认本轮抖动、接触或保持额外位移，相关物理观察保留为空值。

## 控制和实验对应

- 版本 `seven-axis-board-grasp-v33-quasistatic-support`；共同模型、TCP、棋盘变换、Kp/Kd 均未改变。
- 普通 Clearance 保留 shoulder_rotate 跳过逻辑；接近与下降使用新几何求解，端点约束为用户认可的 A8_take，不回放保存的中间角度。
- 请求的模型坐标为 X11.79805、Y405.60068、Z84.62267 mm。这是当前模型对参考端点的 FK，不等于已验证名义 A8 中心坐标的任意 XYZ 求解。
- 13 项准备源码/模型指纹均与实际运行匹配，18 项执行审计全部通过；4471 个完整电机帧、11 个有效静态窗口均可复算，0 个确认振荡事件、0 个故障保持帧、0 个日志丢帧。
- 终点最大关节误差由旧 v32 的 2.5791°降至0.4590°；同串口保持5.0218秒。软件正常完成不等于物理抓取通过。

## 高度原因与下一步

相同模型预测，旧轮带静态跟踪误差的端点高度65.55425 mm，本轮回正后的端点84.44678 mm，增加18.89254 mm；保持阶段只再增加0.44325 mm。该数值是编码器 FK 预测，不能代替实物测量，但与用户观察到更高的方向一致。保留此前错误平衡点不能作为高度修复。

A8_take 捕获的是编码器反馈，缺少捕获时的速度、力矩、增益、开度和接触高度。它的模型高度84.62 mm不能直接等同于正确的实物夹持高度。

已离线研究将请求高度降低25 mm至59.62267 mm，保持原XY；保持完整原朝向的17个初值尚未求得合格解，最好位置残差约8.96 mm。附近目标朝向及单独放宽肩前数学下界的诊断也未得到合格执行候选。有限搜索失败不是物理不可达证明。没有修改运行边界、验收公差、模型补偿或继续执行动作。

后续先继续在共同几何模型内查找可用的低位姿态与必要的共同范围，完整预计算通过后再实测；不添加每格专用偏移，不降低验收标准放行失败路径。

## 文件索引

- [trial_summary.json](trial_summary.json)：代码、请求、物理观察、软件结果、归档及下一次学习文件基线。
- [execution_audit.json](execution_audit.json)：完整18项运行审计，逐点反馈与静态承重估计/已施加/待施加值。
- [height_diagnosis.json](height_diagnosis.json)：新旧姿态、独立FK核对、逐轴高度贡献与保持变化。
- [lower25_endpoint_diagnostic.json](lower25_endpoint_diagnostic.json)、[height_followup_feasibility.json](height_followup_feasibility.json)：低位目标的纯离线诊断，无执行资格。
- [download_verification.json](download_verification.json)：控制包3,834,620字节、RGB-D包87,788,902字节，702对RGB-D，全部大小/SHA/CRC/成员SHA已校验。
- `control_evidence/`：冻结的代码、模型、原始运动日志和连续控制帧；原始档案不随后续分析改写。
- `reproducibility/`：准备、独立审计、诊断与记录脚本；审计依赖已放在同目录，可在Mac继续复核。

本轮仅Clearance学习文件发生正常更新；5份最新学习文件已自动同步回本机。下一次预计算应采用本轮记录的新哈希，不使用上轮Home之前的旧学习基线。
''',encoding='utf8')
print(json.dumps(dict(record=str(registry),sha256=hashlib.sha256(text.encode()).hexdigest(),physical_validation_accepted=False,additional_motion=False)))
