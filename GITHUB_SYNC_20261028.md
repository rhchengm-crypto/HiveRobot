# GitHub 同步记录 — 2026-10-28

## 基线与范围

- 仓库：`https://github.com/rhchengm-crypto/HiveRobot`
- 本次核对时，本地 main 和远端 main 的最后提交均为 `69561db1e68bacdfff5c3bd55d68e2b4f8ebb01e`（2026-09-17，Add confirmed C4 vision routing and rook A1 anchor）。
- 用户要求将其后的全部最新代码同步GitHub。本次将本地运行代码、建模/分析工具、测试、模型JSON、部署说明、交接记录及相关运行证据一并归档。
- 包含视觉/棋冠高度页面更新，以及左臂七轴刚体模型、异步动力学、通用关节目标、棋盘IK路径、承重故障支持、同串口Model Hold、Web集成和相机审计工具。
- 排除临时目录、下载的Python依赖及wheel、原有忽略的缓存/zip、已废弃的生成姿态STEP，以及生成的相机图像/视频/CAD网格数组。这些大体积原始工件保留本地和Jetson，路径见记录索引；源码、模型、结构化分析与完整运行文本日志纳入Git。认可的新版STEP来源和SHA-256详见handoff。
- 对scripts源码、模型/运行记录和outputs归档设置`-text`，保留原始换行字节，避免Git跨平台换行转换破坏模型绑定与部署审计的SHA-256。
- `outputs/` 中的旧版本、撤回试验和实机调度脚本是历史证据，不能作为当前部署入口批量执行。

## 当前实机行为与限制

- 当前board build：`seven-axis-board-grasp-v21-restored-endpoint-settle`。
- 旧endpoint_settle已恢复；最新完整试验因末端关节误差低于0.35°而实际修正0次。
- D向8mm下降补偿已撤回，未进行带该补偿的实机下降。
- 用户观察到的下降朝B偏约10mm仍未解决；本次同步不代表完成任意点物理精度标定。
- shoulder_rotate Kp上限36、Kd4.5，同串口接管、自适应学习和普通Clearance跳过肩旋的逻辑保留。
- 机械臂已Home并暂停。本次GitHub同步只操作本机仓库，不连接串口、不发实机动作、不重启Jetson服务。

## 记录入口与数据版本

- [HANDOFF_20261027.md](HANDOFF_20261027.md) 从上一份2026-09-17交接继续记录，含版本沿革、现场反馈、未完成事项和最终Home状态。
- `outputs/handoff_20261027/records_index.json` 保存1662个相关工件库存和110个源文件/文档/根JSON校验值。
- 最新现场数据在 `outputs/handoff_20261027/jetson_snapshot/data/`，原始快照及本地差异清单也已归档。
- 三个运行模型与本地data副本一致；有8份现场学习/配置JSON与本地旧副本不同，详见handoff及 `local_vs_jetson_json.json`。不要把仓库的整个scripts/data覆盖Jetson，也不要把历史快照当作默认运行配置。

## 离线验证

- 本次执行22个测试模块，覆盖左臂控制、动力学、棋盘IK、承重支持、保持接管、Web路由、几何导入及英文界面。
- 首次270项中269项通过，一项几何导入测试假定运行时JSON必含历史A1锚点。改为逐条核对本次bundle实际提供的所有锚点及其验证元数据，没有修改模型或补回历史姿态。
- 最终测试结果见 `outputs/github_sync_20261028_validation.json` 和同名tests.log。
- 推送后的提交号与远端一致性通过git核对，提交号在交付消息中给出，避免在提交内容中自引用本提交哈希。
