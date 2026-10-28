# 棋盘任意 XY 点抓取：名义几何测试版

这次新增的是任务空间几何求解，底层仍使用现有七轴运动和动力学补偿。
输入连续的棋盘 X、Y，不按动作名称查找抓取姿态。保存姿态只用作 IK 种子和工具朝向参考。

## 当前实际完成程度

- 新版 STEP SHA256：`f7729567ef511d1d9dc38b250dcd276ab09b7aa84fff98f49c0e068971ef9718`。
- 名义 TCP 定义为夹爪安装中心线与远端指尖平面的交点，距 wrist 轴 201.224 mm。
  这是可复算的 CAD 基准，不是 STEP 内语义标注的夹持接触中心；指垫实际接触位置仍需上机核对。
- 固定此 TCP 后，使用现有 C4、D4、C1、B1 四条记录拟合刚性棋盘变换；残差 7.67–19.15 mm。
  JSON 明确保留名义状态，没有把该拟合标成已独立验证。
- 操作范围来自当前保存姿态和 Home/Clearance 的范围外扩 10°，不是厂家机械限位。
- 离线完整路径扫描：64 格中心中 33 个求解成功。其余在当前朝向、操作范围和路径条件下失败；
  这既不证明它们物理不可达，也不能宣称全棋盘覆盖完成。
- 规划检查采样连杆轴包络与有限棋盘平面，不包含其它棋子、完整指爪网格或自碰撞规划。
  首轮使用清理过的测试区域。

## 同步文件

本地 `scripts/` 下这四个文件同步到 Jetson 对应相对位置：

```text
/home/nvidia/hive_robot/DM_Control_Python/left_arm_board_grasp.py
/home/nvidia/hive_robot/DM_Control_Python/left_arm_board_kinematics.py
/home/nvidia/hive_robot/DM_Control_Python/chessboard_vision_v3_0_web_control.py
/home/nvidia/hive_robot/DM_Control_Python/data/left_arm_board_grasp.json
```

依赖已部署的 `left_arm_general_motion.py`、`left_arm_rigid_body.py`、
`left_arm_general_move.py`、`left_arm_v3_1_model_control.py`、`left_arm_v2_6.py`、
`left_arm_v2_8_move_library.py`。通用动力学 JSON SHA256 必须仍是
`83f5e4b240d48ce4aae1c8037922696c5f2f1ed120476c8ae7da9e6ca00d6fd7`。
新入口会检查该哈希，避免几何配准与动力学几何不一致。
不要覆盖 Jetson 上的自适应学习、保存姿态、历史采样文件。

## 完整启动命令

结束旧 Web 服务后，只启动一个服务：

```bash
cd /home/nvidia/hive_robot/DM_Control_Python
python3 -u chessboard_vision_v3_0_web_control.py \
  --host 0.0.0.0 --port 8100 --python-bin python3 \
  --arm-script left_arm_v2_8.py \
  --move-script left_arm_v3_0_move_library.py \
  --enable-execute \
  --dynamics-model data/left_arm_v3_1_dynamics.json \
  --motion-dynamics \
  --rigid-body-model data/left_arm_general_dynamics.json \
  --rigid-body-mode apply
```

## 第一轮实机操作

1. 在原 Web 界面执行普通 Table Clearance，等动作及 Model Hold 结束。
   不需要再重复 Home 验收。普通 Clearance 的 shoulder_rotate 跳轴逻辑未改。
2. 打开 Dynamics Control，找到 **Board Point Grasp**。
3. 首个点选 C4：X=`137.5`、Y=`192.5`，profile=`white_knight`，stage=`Above Only`。
   棋盘外框 a1 角是原点，X 朝 h1、Y 朝 a8；单位 mm。
4. 点 **Preview Board Path**。此操作读取当前位置并求解整条路径，不使能电机、不移动、不自动接管。
   确认返回 `ok: true` 和规划采样数量。如果报 IK/路径错误，不执行，保留完整错误。
5. 确认路径附近没有手或其它障碍物，勾选 Enable actual motor commands，点 **Run Board Path**。
   这一阶段只到名义 C4 上方 160 mm，不下降到棋子、不驱动夹爪；随后沿用原有 5 秒 Model Hold。
6. 保存完整输出和下面的摘要。名义配准尚有厘米级残差，首轮先核对这次上方运动与接管，
   不把这轮当成抓取精度验收。需要现场修正时应统一更新棋盘/TCP 配准，不给单个动作调增益。

```bash
python3 left_arm_general_motion.py --summary --action dynamics:board-grasp --last 3
```

后续阶段：Touch Position 下降到所选夹持高度但不驱动夹爪；Pressure Grasp and Lift 会先开爪，
到抓取点后复用原有压力/堵转接触检测，未确认接触就终止，不进入提起；提起时持续刷新夹爪保持命令。
应在完成名义坐标的实物核对后测试这两个阶段。

现有白象、白马的夹持高度记录为 55 mm。白车配置沿用已测试 C4 白车流程使用的白马抓取朝向/高度，
不是新增了独立高度测量。兵、后等未建立完整夹持配置的棋子不会偷偷套用默认高度。

## 命令行预览和运动

读取真实当前位置、只规划、不运动：

```bash
python3 -u left_arm_board_grasp.py \
  --model data/left_arm_general_dynamics.json \
  --board-model data/left_arm_board_grasp.json \
  --x-mm 137.5 --y-mm 192.5 --piece white_knight --stage above \
  --plan-output data/board_grasp_preview.json
```

核对后在同一命令末尾加 `--execute` 才运动。命令行默认同样运行原有 Model Hold。
连续坐标示例为 `--x-mm 139.2 --y-mm 194.7`，不需要先保存一个新动作。

## 保留行为

- 原有 Home、Clearance、点动、保存动作的学习代码、数据和参数未改。
- 新入口成功执行后走现有 Model Hold 当前状态桥接；预览或失败不会启动成功接管。
- 新轨迹复用已有连续 group 控制器和 ±0.5 Nm 通用动力学补偿，未增加补偿上限。
- 新点不伪造历史标定样本，不写入旧动作的学习记录。
