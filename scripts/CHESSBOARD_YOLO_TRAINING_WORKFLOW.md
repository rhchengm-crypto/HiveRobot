# Chessboard YOLO Training Workflow

本文档记录 HiveRobot 棋盘视觉 v2.7 的 YOLO 棋子识别训练流程。当前设计是：

```text
live RGB/depth frame -> whole-board occupancy -> YOLO piece class -> square identity -> opening placement plan
```

几何和深度检测负责判断哪些格子有棋子；YOLO 负责把已占用格子识别成 `white_queen`、`black_king` 等具体棋子类别。

## 1. 类别定义

当前模型固定 12 类：

```text
white_pawn
white_rook
white_knight
white_bishop
white_queen
white_king
black_pawn
black_rook
black_knight
black_bishop
black_queen
black_king
```

同类棋子不区分唯一编号。例如 8 个白兵都标为 `white_pawn`，后续开局摆放逻辑会把任意 `white_pawn` 分配到空的白兵目标格。

## 2. 训练前准备

在 Orin 上进入项目目录：

```bash
cd /home/nvidia/hive_robot/DM_Control_Python
```

启动 v2.7 Web verifier：

```bash
python3 scripts/chessboard_vision_v2_7_web_control.py --port 8097
```

Windows 浏览器打开：

```text
http://<orin-ip>:8097/
```

采集训练数据前先确认：

```text
has_depth: true
rgb_seq 持续增加
depth_seq 持续增加
depth_age_s 和 rgb_age_s 不要太大
黄色棋盘网格与真实棋盘内侧黑线对齐
```

如果网格不准，先调整四个棋盘角点。角点顺序必须是：

```text
a1, h1, h8, a8
```

这里的角点是棋盘 8x8 区域的内侧黑线角点，不是外圈坐标装饰边。

## 3. 推流步骤

YOLO 训练和 whole-board 验证都依赖实时 RGB/depth 数据。现场推荐先启动 HP60C ROS 驱动，再启动 v2.7 Web verifier。

第一个 Orin 终端启动 HP60C：

```bash
cd ~/ascam_ws
source ~/.bashrc
roslaunch ascamera hp60c.launch
```

保持这个终端不要关闭。看到 RGB/depth topic 正常发布后，在第二个 Orin 终端启动棋盘视觉 Web verifier：

```bash
cd /home/nvidia/hive_robot/DM_Control_Python
source ~/.bashrc
python3 scripts/chessboard_vision_v2_7_web_control.py --port 8097
```

终端启动成功后会打印：

```text
Live RGB stream: http://<orin-ip>:8097/live-rgb.mjpg
Live overlay stream: http://<orin-ip>:8097/live-overlay.mjpg
```

Windows 浏览器打开主页面：

```text
http://<orin-ip>:8097/
```

也可以直接打开两个 MJPEG 推流地址：

```text
http://<orin-ip>:8097/live-rgb.mjpg
http://<orin-ip>:8097/live-overlay.mjpg
```

两个流的用途：

```text
/live-rgb.mjpg      原始 RGB 画面，用来确认相机画面是否实时、清晰、无遮挡
/live-overlay.mjpg  棋盘网格、检测点、棋子类别标签叠加画面，用来验证 whole-board + YOLO 接入
```

主页面里常用操作顺序：

```text
1. 页面初始显示 /live-rgb.mjpg。
2. 点击 Inspect Square 或 Detect Whole Board 后，会切到带网格的 overlay。
3. 点击 Show Input Frame 可回到原始 live RGB。
4. 点击 Refresh Live Status 查看 rgb_seq、depth_seq、rgb_age_s、depth_age_s。
```

推流正常的判断标准：

```text
rgb_seq 持续增加
depth_seq 持续增加
rgb_age_s 通常小于几秒
depth_age_s 通常小于几秒
live RGB 中棋盘没有严重过曝、虚焦或遮挡
live overlay 中黄色网格与真实棋盘 8x8 内侧黑线对齐
```

如果浏览器打不开：

```text
确认 Orin 和 Windows 在同一网络
确认 URL 使用 Orin IP，不是 localhost
确认 8097 端口没有被其他进程占用
确认 Web verifier 终端没有报错退出
```

如果有 RGB 但没有 depth：

```text
检查 HP60C ROS driver 是否还在运行
检查 /ascamera_hp60c/depth0/image_raw 是否发布
启动 Web verifier 时显式指定 depth topic
```

显式指定 topic 的启动方式：

```bash
python3 scripts/chessboard_vision_v2_7_web_control.py --port 8097 \
  --rgb-topic /ascamera_hp60c/rgb0/image \
  --depth-topic /ascamera_hp60c/depth0/image_raw
```

旧的 `hp60c_stream_server.py` 仍可用于手臂目标调试，默认端口是 `8090`：

```bash
cd ~/hive_robot/DM_Control_Python
source ~/.bashrc
python3 scripts/hp60c_stream_server.py --port 8090
```

但棋盘 YOLO 训练、采样、whole-board 验证请优先使用 v2.7 Web verifier 的 `8097` 页面。

## 4. 数据集目录

默认数据集目录：

```text
datasets/chess_pieces_yolo
```

初始化数据集：

```bash
python3 scripts/chess_piece_yolo_dataset.py init \
  --dataset-dir datasets/chess_pieces_yolo
```

初始化后结构应该是：

```text
datasets/chess_pieces_yolo/
  data.yaml
  images/
    train/
    val/
    test/
  labels/
    train/
    val/
    test/
```

`data.yaml` 会写入上面的 12 个类别，并指向 `images/train`、`images/val`、`images/test`。

## 5. 推荐采样方法

推荐把棋子放在第 4 横排：

```text
a4,b4,c4,d4,e4,f4,g4,h4
```

原因：

```text
第 4 横排离棋盘边缘较远，误检较少
8 个格子可以一次采 1-8 个棋子
YOLO 标注由格子 ROI 自动生成，不需要手工画框
```

每个棋子尽量放在格子中心附近。训练集要覆盖这些变化：

```text
不同棋子类别
白棋和黑棋
不同格子位置
棋子在格子中心附近的轻微偏移
正常光照、偏暗、偏亮
手臂或夹爪靠近但不遮挡棋子时的场景
单个棋子、多枚棋子混合摆放
```

建议第一版最少采集量：

```text
每类 30-50 张 train
每类 8-15 张 val
总量大约 400-800 个棋子框
```

后续如果某类容易混淆，比如 queen/king 或 bishop/pawn，再针对性补样本。

## 6. 用 Web UI 采集样本

在网页中操作：

```text
1. 把棋子放到第 4 横排，例如 d4 放白后。
2. 在 YOLO rank-4 labels 输入框填写标签。
3. YOLO split 填 train。
4. 点击 Save YOLO Sample。
5. 更换棋子组合、位置或光照后重复保存。
6. 留出一部分样本，把 YOLO split 改成 val。
7. 再保存验证集样本。
```

标签支持一行一个：

```text
a4:white_pawn
b4:white_rook
c4:black_king
d4:white_queen
```

也支持逗号分隔：

```text
a4:white_pawn,b4:white_rook,c4:black_king,d4:white_queen
```

注意事项：

```text
只填写真实放了棋子的格子
格子名必须是 a1-h8
类别名必须完全匹配 12 类定义
train 和 val 不要用完全相同的图片
```

Web server 会保存当前 live RGB 图，并根据当前棋盘标定自动生成 YOLO 格式标注。

## 7. 用 CLI 手动添加样本

如果已有图片，也可以手动加入数据集：

```bash
python3 scripts/chess_piece_yolo_dataset.py add-image \
  --image /tmp/chess_rank4_001.jpg \
  --calibration scripts/data/chessboard_vision_v2_7_calibration.json \
  --dataset-dir datasets/chess_pieces_yolo \
  --split train \
  --placements "a4:white_pawn,b4:white_rook,c4:black_king,d4:white_queen"
```

如果要指定输出样本名：

```bash
python3 scripts/chess_piece_yolo_dataset.py add-image \
  --image /tmp/chess_rank4_002.jpg \
  --calibration scripts/data/chessboard_vision_v2_7_calibration.json \
  --dataset-dir datasets/chess_pieces_yolo \
  --split val \
  --name rank4_val_002 \
  --placements "d4:white_queen"
```

标注框默认使用格子 ROI 的缩小框：

```text
--shrink 0.72
```

如果框太大，容易把格子边线和邻格纹理学进去；如果框太小，会切掉棋子外形。第一版保持默认即可。

## 8. 训练前检查数据集

检查 `data.yaml`：

```bash
cat datasets/chess_pieces_yolo/data.yaml
```

检查图片和标签数量是否对应：

```bash
find datasets/chess_pieces_yolo/images/train -type f | wc -l
find datasets/chess_pieces_yolo/labels/train -type f | wc -l
find datasets/chess_pieces_yolo/images/val -type f | wc -l
find datasets/chess_pieces_yolo/labels/val -type f | wc -l
```

抽查标签文件：

```bash
head datasets/chess_pieces_yolo/labels/train/*.txt
```

每一行应该是 YOLO 格式：

```text
class_id center_x center_y width height
```

所有坐标都是 0-1 之间的归一化数值。

## 9. 从 Web UI 启动训练

网页训练流程：

```text
1. 确认已经有 train 和 val 样本。
2. 点击 Start YOLO Train。
3. 点击 YOLO Train Status 查看日志。
4. 训练异常或想重跑时，点击 Stop YOLO Train。
```

训练日志：

```text
/tmp/hive_robot_chessboard_vision_v2_7/yolo_train.log
```

默认输出模型：

```text
runs/chess_piece_yolo/yolo11n_rank4/weights/best.pt
```

Web server 的训练策略：

```text
如果 host 上有 yolo 命令，直接调用 host yolo
如果 host 上没有 yolo 命令，自动使用 Docker fallback
Docker 镜像默认是 ultralytics/ultralytics:latest-jetson-jetpack5
```

默认训练参数：

```text
model=yolo11n.pt
imgsz=960
epochs=120
batch=8
project=runs/chess_piece_yolo
name=yolo11n_rank4
```

## 10. 手动 Docker 训练

如果想手动进入容器训练：

```bash
sudo docker run -it --rm \
  --runtime nvidia \
  --network host \
  --ipc host \
  -v ~/hive_robot:/workspace/hive_robot \
  ultralytics/ultralytics:latest-jetson-jetpack5
```

容器内执行：

```bash
cd /workspace/hive_robot/DM_Control_Python
yolo detect train \
  model=yolo11n.pt \
  data=datasets/chess_pieces_yolo/data.yaml \
  imgsz=960 \
  epochs=120 \
  batch=8 \
  project=runs/chess_piece_yolo \
  name=yolo11n_rank4
```

如果显存或内存不够，先把 batch 降低：

```bash
yolo detect train \
  model=yolo11n.pt \
  data=datasets/chess_pieces_yolo/data.yaml \
  imgsz=960 \
  epochs=120 \
  batch=4 \
  project=runs/chess_piece_yolo \
  name=yolo11n_rank4
```

## 11. 训练结果检查

训练完成后检查模型文件：

```bash
ls -lh runs/chess_piece_yolo/yolo11n_rank4/weights/best.pt
```

查看训练输出图表：

```bash
ls runs/chess_piece_yolo/yolo11n_rank4
```

重点看：

```text
results.png
confusion_matrix.png
val_batch*_pred.jpg
```

第一版目标不是追求极高 mAP，而是确认：

```text
常见棋子能稳定识别
white/black 不混
king/queen/bishop/pawn 不频繁互相混淆
误检空格子的概率低
```

## 12. CLI 推理验证

对单张图片运行推理：

```bash
python3 scripts/chess_piece_yolo_infer.py \
  --model runs/chess_piece_yolo/yolo11n_rank4/weights/best.pt \
  --image /tmp/chess_rank4_test.jpg \
  --calibration scripts/data/chessboard_vision_v2_7_calibration.json \
  --squares a4,b4,c4,d4,e4,f4,g4,h4
```

降低置信度做排查：

```bash
python3 scripts/chess_piece_yolo_infer.py \
  --model runs/chess_piece_yolo/yolo11n_rank4/weights/best.pt \
  --image /tmp/chess_rank4_test.jpg \
  --calibration scripts/data/chessboard_vision_v2_7_calibration.json \
  --squares d4 \
  --conf 0.15
```

输出中重点检查：

```text
detections
piece_class_results
placement_plan
```

正常示例：

```text
piece_class_results.d4.piece_class = white_queen
piece_class_results.d4.confidence 大于 0.8
placement_plan 包含 pick=d4, place=d1
```

如果需要告诉摆放规划哪些目标格已经占用：

```bash
python3 scripts/chess_piece_yolo_infer.py \
  --model runs/chess_piece_yolo/yolo11n_rank4/weights/best.pt \
  --image /tmp/chess_rank4_test.jpg \
  --calibration scripts/data/chessboard_vision_v2_7_calibration.json \
  --squares d4,e4 \
  --occupied-targets d1,e1
```

## 13. Web UI 推理验证

网页中验证当前 live frame：

```text
1. 把训练好的 best.pt 路径填入 YOLO model。
2. YOLO detect squares 填要检测的格子，例如 d4。
3. 点击 YOLO Detect Table。
4. 查看 YOLO result table 和下方 JSON。
```

期望结果：

```text
Square = d4
Piece = white_queen
Conf 接近 0.9 或更高
Place = d1
```

如果 `YOLO Detect Table` 可以识别，但 `Detect Whole Board` 不能识别，优先检查 whole-board 是否把 occupied square 传给了 YOLO。

## 14. Whole-board 接入验证

集成检测使用 `Detect Whole Board`，不是普通的 `Inspect Square`。

当前流程：

```text
1. Full-board depth/RGB occupancy 检测 64 个格子。
2. 得到 detected_squares，例如 d4。
3. server 用干净的原始 RGB frame 运行 YOLO。
4. YOLO 检测框中心映射回 allowed squares。
5. yolo_identified_pieces 覆盖 unknown_piece。
6. JSON 和 live overlay 都显示 white_queen 等类别。
```

直接 API 示例：

```text
/api/inspect?...&squares=all&run_yolo=1&yolo_model=/home/nvidia/hive_robot/DM_Control_Python/runs/chess_piece_yolo/yolo11n_rank4/weights/best.pt
```

正常 JSON 摘要：

```text
detected_count=1
detected_squares=d4
identified_pieces=d4:white_queen
yolo_result.detection_count=1
yolo_result.mapped_square_count=1
```

正常完整字段：

```text
identified_pieces.d4.piece_id = white_queen
identified_pieces.d4.identity_method = trained_yolo_model
identified_pieces.d4.detection_method = yolo
yolo_identified_pieces.d4.piece_id = white_queen
piece_results.d4.piece_id = white_queen
```

`Detect Whole Board` 成功后，live overlay 会缓存最新 YOLO 身份，视频流应显示：

```text
detected=d4
white_queen
```

## 15. 空棋盘基线配合

空棋盘基线用于抑制固定位置的误检，比如 rank 1 边缘深度噪声或木纹 RGB 误检。

采集方法：

```text
1. 清空棋盘。
2. 保持相机、棋盘和光照与训练/运行时一致。
3. 在 Web UI 点击 Capture Empty Board Baseline。
4. 再点击 Detect Whole Board 验证 detected_count 接近 0。
```

默认保存路径：

```text
data/chessboard_vision_v2_7_empty_board_baseline.json
```

如果移动了相机、棋盘或明显改变光照，建议重新采集空棋盘基线。

## 16. 常见问题

### Whole-board 还是 unknown_piece

检查：

```text
是否点击 Detect Whole Board
请求是否包含 run_yolo=1
YOLO model 路径是否指向存在的 best.pt
JSON 里是否有 yolo_result
yolo_result.detections 是否为空
yolo_result.piece_class_results 是否为空
```

如果 `detections` 为空：

```text
模型可能没有加载到
当前画面和训练数据差异太大
置信度阈值太高
YOLO 输入图片可能不干净
```

可以用 `YOLO Detect Table` 单独验证同一个格子。

### detections 有框，但 piece_class_results 为空

说明 YOLO 找到了棋子，但检测框中心没有映射到允许的格子。

检查：

```text
棋盘角点标定
YOLO detect squares 或 whole-board detected_squares
检测框中心是否落在目标格 ROI 内
```

### JSON 正确，视频流还是 unknown

检查：

```text
是否重启了 Web verifier
浏览器是否刷新
是否至少点击过一次 Detect Whole Board
live overlay 是否拿到 latest_yolo_identified_pieces 缓存
```

### 邻格误检，例如 d4 棋子旁边多出 d5

当前逻辑会在 YOLO 身份缓存存在时抑制相邻 unknown artifact。处理步骤：

```text
1. 先点击 Detect Whole Board，让 YOLO 正确识别 d4。
2. 刷新视频流观察 overlay。
3. 如果 d5 仍出现，重新采空棋盘基线，并补充该场景训练样本。
```

### 某个棋子类别总是混淆

优先补数据，不要先调代码：

```text
只采容易混淆的类别
保持同样棋盘、同样相机视角
每类补 20-40 个样本框
train 和 val 都补
重新训练或 fine-tune
```

## 17. 推荐迭代节奏

第一轮：

```text
每类 30-50 个 train 框
每类 8-15 个 val 框
训练 yolo11n_rank4
用 YOLO Detect Table 验证 rank 4
用 Detect Whole Board 验证 d4/e4 等单子场景
```

第二轮：

```text
记录混淆类别和失败画面
补充失败场景样本
重新训练为 yolo11n_rank4-2
把 Web UI 的 YOLO model 指向新的 best.pt
再次验证 whole-board JSON 和 live overlay
```

第三轮：

```text
扩展到更多棋盘位置
增加真实开局摆放和抓取前后场景
加入手臂靠近、阴影、轻微遮挡样本
固定一版稳定模型用于自动摆棋
```

当前已验证的正确集成输出形态：

```text
detected_count=1
detected_squares=d4
identified_pieces=d4:white_queen
yolo_identified_pieces.d4.piece_id=white_queen
yolo_result.detection_count=1
yolo_result.mapped_square_count=1
```
