# 完整棋子标注修复

2026-09-07：版本 `v2.7-grid-boxes-yolo-occupancy-fusion` 移除 Inspect 中
“深度先检出才运行 YOLO”的前置条件。运行 YOLO 时对所选全部格子映射检测，
成功识别的棋子可补回深度漏检，原深度结果保存在 `geometry_detection`。
同步 `chessboard_vision_v2_7_web_control.py` 后重启生效，无需重新训练。
仍保留无 YOLO 身份的深度候选及高棋子投影核对逻辑。

最终显示版本为 `v2.7-full-board-crown-projection-v2-grid-boxes`。
Inspect Square（勾选 Detect pieces）、Detect Whole Board 和 YOLO Detect Table
在同次静态识别图上同时绘制黄色网格、绿色预测框、类别和置信度。
这一显示更新需同步 `chess_piece_yolo_infer.py` 与 `chessboard_vision_v2_7_web_control.py`。
下文投影逻辑仍沿用 v2；显示更新不需要重新训练。

## 全棋盘高棋子投影重复检测修正

版本 `v2.7-full-board-crown-projection-v2`：更新同目录中的
`chessboard_vision_v2_7.py` 和 `chessboard_vision_v2_7_web_control.py`，
重启 Web 服务即可，继续使用现有 best.pt，不需要重新训练。

整盘检测在本次 YOLO 输出后，再核对深度候选。对于稳定棋子预测框上半部内的
低票数、高深度凸起候选，记录 `yolo_crown_depth_projection` 并排除，
同时更新 piece_results、detected_squares、detected_count、identified_pieces。
规则不限定 d 列或第 4/5 排，使用棋盘邻接关系和图像坐标。
独立 YOLO 检出的相邻棋子和稳定深度候选不会被此规则删除。
v2 将固定少于 3 票改为严格少于半数采样帧，同时要求主体超过半数帧检出。
因此 3/7 的 e6 投影会被核对排除，3/5、4/7 的候选仍保留。
已回放 d5/d6 与 e5/e6 两份现场记录，并覆盖不同采样窗口的边界测试。
实时叠加中的历史身份缓存不再用于“一律删除相邻格子”。

已用提供的 d5 白王/d6 假影记录回放验证，并测试所有格子的相邻方向。
这不代表实物全棋盘已验证；部署后需测试单枚白王在不同位置，以及两枚棋子真实相邻。
稳定出现但证据不足的候选会保留，避免仅凭 YOLO 漏检就误删真实棋子。

## 识别结果显示预测框

更新 `chess_piece_yolo_infer.py` 和 `chessboard_vision_v2_7_web_control.py` 后，
重启原 Web 服务。此显示功能不需要重新训练模型。
点击 Detect Whole Board 或 YOLO Detect Table 后，主画面显示该次输入的静态识别图：
绿色预测框、模型预测类别、置信度。绿色不表示识别正确。
点击 Show Input Frame 返回实时无框画面。

原始输入 `image_path` 不变；相同目录另存 `_pred.jpg`，JSON 增加
`prediction_image_path` 和 `prediction_image_url`。
框使用原始 `detections.bbox_xyxy`，包括未成功映射到格子的检测；
不会把上一次预测框贴到正在变化的实时视频上。

2026-09-06：旧采样器按格子缩小到 72% 生成标签，可能漏掉棋冠。
新采样器向上扩展参考框，但这只是待复核草稿，不是自动轮廓识别。
已有原图保留，通过原图上的拖框编辑修正 train、val；不要编辑 train_batch 拼图。

## 部署到 Orin

需要把以下四个文件放到 Orin **实际正在运行的 Web 脚本所在目录**，替换前备份旧代码：

- chessboard_vision_v2_7_web_control.py
- chess_piece_yolo_dataset.py
- chess_piece_yolo_infer.py
- chess_piece_yolo_labels.py（新增）

使用原启动命令、原参数重启 Web verifier，保留原来的数据集目录参数。
如果不确定正在运行哪个脚本，可在 Orin 终端执行 `pgrep -af chessboard_vision_v2_7_web_control`。
本地镜像没有训练原图，本次未修改 Orin 标签、未启动训练。

## 修复和训练

1. 停止训练。在 8097 主页面点击 **Review YOLO Labels / 复核完整棋子框**，
   或访问 `http://<orin-ip>:8097/yolo-labels`。
2. 下拉列表显示所有 train、val、test 原图及复核状态，优先打开待复核图片。
3. 选中要修的框，在原图上从棋冠左上方拖到完整底座右下方。
   框应贴近整个可见棋子，可以跨格，不能只围住底座。
4. 检查类别；有多枚棋子时逐一修框。可新增遗漏的框或删除错误的框。
5. 点击 **确认完整并保存 → 下一张**。旧标签备份在数据集 `label_backups/`，
   新标签写入原 `labels/<split>/`，对应 split 的标签缓存自动清除。
6. train 和 val 全部复核后，回主页面点击 **Start YOLO Train**。
   Web 训练会拒绝未复核标签；后续修改原图或标签会使复核状态失效。
7. 训练完成后选择新 best.pt，测试 d4、d5 及其他未参与训练的位置。

直接用 CLI / Docker 训练会绕过 Web 检查，启动前先在相同数据集上执行：

```bash
python3 /实际脚本目录/chess_piece_yolo_labels.py --dataset-dir /实际数据集目录
```

不要把数据增强造成的旋转拼图当作原图来修标注；编辑器读取原始 images 文件和同名 labels 文件。
空棋盘负样本可保存空标签，前提是画面确实没有需标注棋子。

## 格子映射变化

完整框的中心可能投影到后方邻格。映射改用框高度 90% 处的水平中点作为近似底座点，
适用于现场这种“棋冠在上、底座在下”的相机画面。
原框中心保留在 `bbox_center_px`；映射结果 `center_px` / `center_mm` 对应该底座近似点。
这是格子归属估计，不是经过现场验证的精确抓取坐标。相机倒置或视角明显变化时需重新验证。
原始 `detections.center_px` 仍是框中心。

## 本地验证

```powershell
python -m unittest discover -s scripts -p test_chess_piece_yolo_labels.py -v
```

覆盖草稿向上扩展、未复核训练拦截、备份、缓存清除、过期写入拒绝、
原图修改使复核失效、非法框/路径拒绝、完整框的 d5 归属及同格置信度去重。
