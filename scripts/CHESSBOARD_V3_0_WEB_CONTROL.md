# HiveRobot v3.0 C4 识别确认与多流程路由

入口脚本：`chessboard_vision_v3_0_web_control.py`，默认端口 `8100`。

v3.0 通过导入 v2.8 保留棋盘识别、训练、机械臂控制、Saved Moves、多流程、
标定及全部既有数据。v2.7 和 v2.8 文件没有被修改。

## 新流程

机械臂页面新增“识别 C4 并确认执行”按钮：

1. 固定使用当前 YOLO 模型识别 `c4`，此阶段不会启动机械臂。
2. 若未识别到棋子或识别为 pawn、queen、king 等其他类别，只显示结果，
   不生成执行授权，机械臂不动作。
3. 识别为 bishop 时，弹窗显示预测图、类别、置信度以及
   `White Bishop Placement`；确认后先使用已验证的 saved move `white_knight_c4`
   夹取 C4，再沿用既有 bishop 放置动作放到 C1。
4. 识别为 knight 时，弹窗显示预测图、类别、置信度以及
   `white knight place_B1`；确认后使用 saved move `white_knight_c4` 启动该既有多流程。
5. 识别为 rook 时，同样先使用 `white_knight_c4` 的 C4 抓取与 Clearance，随后
   执行 saved move `white_rook_place_A1` 放到 A1，最后张爪。
6. 取消或关闭确认不会启动机械臂。确认使用服务器端一次性令牌，120 秒后失效，
   且每个令牌只能使用一次。

v3.0 自动路由遵循页面上的“启用 0.5° 门禁”勾选项：默认不勾选时只记录和
学习误差而不中止流程；勾选后，确认启动的自动多流程也会启用门禁。

`left_arm_v3_0_move_library.py` 为 C4 bishop 和 rook 组合 C4 抓取锚点及各自的
既有放置动作；其他命令原样交给 v2.8 动作库。v2.8 源文件及其训练数据不会改写。

## 启动

在已经加载 ROS 环境的终端运行：

```bash
cd /home/nvidia/hive_robot/DM_Control_Python
python3 chessboard_vision_v3_0_web_control.py \
  --host 0.0.0.0 --port 8100 --enable-execute
```

浏览器打开：

```text
http://192.168.0.199:8100/
```

必须使用 `--enable-execute` 才能在确认后启动机械臂；不带该参数时可以完整测试
C4 识别和确认界面，但执行接口会沿用原有执行禁用保护。

识别使用当前 overlay 配置中的棋盘四角、额外标定点和 YOLO 模型路径。执行前应确认
预测图中的框确实落在 C4 棋子上，并确认弹窗类别与实物一致。
