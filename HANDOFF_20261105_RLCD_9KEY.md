# RLCD 与 9 键灯控交接 — 2026-11-05

日期按 America/Los_Angeles。用户要求今天收尾，将所有相关最新代码同步 GitHub。本文件只交接本次 ESP32 项目，不更改仓库中的机器人项目。

## 当前实板与源码

| 项目 | 电脑上最新工作目录 | 仓库目录 | 端口 |
|---|---|---|---|
| 当前 RLCD 五页完整版 | `C:/Users/rhche/Desktop/rlcd-gpt-tailscale` | `rlcd-gpt-tailscale/` | COM19 |
| RLCD 四页直连版、GPT 集成前基线 | `C:/Users/rhche/Desktop/rlcd-qwen-asr` | `rlcd-qwen-asr/` | 同一块 RLCD；当前实板已是五页版 |
| 双 ESP32 九键灯控最新版 | `C:/Users/rhche/Desktop/dual_esp32_macro` | `dual_esp32_macro/` | 主板历史为 COM4，接板后需重新枚举 |

RLCD 型号：Waveshare ESP32-S3-RLCD-4.2，16MB Flash、8MB OPI PSRAM，MAC `28:84:85:a8:8f:70`。不要把 RLCD 固件刷到旧 COM4 灯控主板。当前最新刷入为新闻页 K1 流式播报版，保留电池图标内百分比；没有改刷灯控板。末节记录后续验证。

## RLCD 当前功能与按键

顶部标签从左到右：**转文字 → 千问 → GPT → 天气 → 新闻**。白底黑字、加粗中文、实时当地时间。

- BOOT / GPIO0：当前功能动作；转文字页开始/停止连续听写，问答页开始录音/结束发送，天气和新闻页手动刷新。
- KEY / GPIO18：短按翻当前内容页，长按约 1 秒切换功能标签。GPT 从 SD 恢复的答案也可翻页。
- RX / GPIO44：短按休眠/唤醒，长按约 3 秒手机网页配网。原按键功能对换已保留。
- 外接四键的实际 GPIO 为 1、2、3、17，历史接线顺序曾调整；最终左右映射需以 `ExternalKeys.h` 和实板为准，不照旧聊天猜接线。

连续上海话转普通话：`qwen-audio-3.1-asr-flash-message`，24kbps Ogg Opus、40ms 帧，`near_meeting_16k`、800ms 静音断句、`keep_dialect=false`。停顿自动出标准普通话，只翻译、不回答；静音不结束整个听写会话，按 BOOT 停止。积压故障及停顿超过 10 秒才出字的问题已修复，用户确认明显更快、文字正常。800ms 是端点阈值，不是端到端出字保证。

千问问答与每日 10 条社会新闻：已升级 **`qwen3.8-flash`**。新闻改用 DashScope multimodal-generation endpoint 和 text content blocks，保留来源、日期、去重校验。千问录音的独立识别模型仍为 `qwen3-asr-flash`，没有一并替换。新闻每天当地 07:00 预先生成，失败保留缓存。

天气：公网 IP 定位城市与时区，显示当地三天；网络出口变化会重新定位，当前网络曾定位 Redmond。不是仅看局域网 IP，也不固定 ZIP 98074。

GPT：**`gpt-6.1-sol`**，低推理强度、4096 combined token budget；录音识别 `gpt-4o-mini-transcribe`。板上先压缩为 Ogg Opus，经原生 MicroLink/WireGuard 到树莓派私有网关，再由树莓派请求 OpenAI。支持真实 web_search，天气/新闻/最新信息要求搜索完成并附来源。

千问与 GPT 内容卡片底部显示模型型号；GPT 从经过认证的 health 读取配置型号，并用实际响应 model 更新。用户确认千问型号清楚、问答正常；GPT 6.1 和联网天气亦已有用户确认。

## 今日最后修改：电池显示

板载 GPIO4 / ADC1 channel 3 已有 200kΩ/100kΩ 三倍分压，无需增加接线。`Battery.cpp/.h` 使用 ESP-IDF oneshot ADC 与 curve-fitting 校准，16 点去极值平均，每 5 秒采样并平滑。

- 右上角电池图标内居中显示 `~96%` 这类估算值，数字在填充/空白区域反色，三位数百分比也能容纳。
- 百分比基于单节 18650 电压曲线估算；负载、温度、充电影响读数，不是库仑计，不推断充电状态；无效采样显示 `--%`。
- 电压低于等于 3.50V 时旁边显示 `LOW`；没有新增自动断电。
- 实测约 **4.11V / 96%**；校准读数稳定，截图确认最后的叠加布局正常。
- 首版电量固件做过自动休眠/唤醒：result=0、mic=1，唤醒后 ADC 有效、SD ready=1。最后叠加版只改布局，编译、刷入哈希校验和帧缓冲视觉检查通过。
- 串口命令 `battery-info`。本地截图 `battery-overlay.png`；日志、截图及含私密信息的固件备份不公开。

## SD、网络与树莓派

SDMMC：CLK38、CMD21、D0=39，一位总线 10MHz；实卡报告约 120000MiB。启动读写校验，不格式化。历史写入 `/rlcd/history/YYYYMMDD.txt`，保存转写和问答文本；`/rlcd/last-gpt.jsn` 恢复上次 GPT 回答，不保存录音。休眠前 flush。文本历史属于用户私人内容，不上传 GitHub。

树莓派：Tailscale `100.71.158.56`，SSH 用户 pi，服务 `rlcd-gpt-gateway`，代码 `/home/pi/rlcd-gpt-gateway/gateway.py`。网关仅绑定 Tailnet IP:8765，Bearer token 认证；OpenAI key 仅在树莓派和电脑本地配置中。

只有进入 GPT 页时启动板上 VPN，离页/休眠停止，返回恢复。转文字、千问、天气、新闻沿用直连 Wi-Fi。**没有把板上默认路由设置成树莓派 exit node**；实现的是经 Pi 网关发 GPT 请求。天气/新闻后台请求在 VPN 退出后继续，避免 TLS 内存峰值。

关键 MicroLink 补丁均需保留：任务退出确认再释放；队列清理；TCPIP 线程卸载 netif；基于同步 epoch 的 TAI64N 重放时间戳；解密包交给 `netif->input/tcpip_input`，不能从 WG worker 直接 `ip_input`；DERP TLS 证书验证；VPN 源 IP 绑定和限时上传。GPT Opus 任务使用 40KiB PSRAM 栈与匹配的 WithCaps 清理，不能退回旧 12KiB 栈。

电脑 Tailscale 收尾保留原始停止状态及原 Pi exit-node 配置。后续如临时启动电脑 Tailnet 做 SSH，完成后恢复原设置。

## 9 键灯控最新版

同步的是桌面 2026-10-25 最终 HOT&TV 中文播报版，不是旧五键版，也不是只有九键测试界面的早期版。主/副板源代码、common 协议、素材生成/传输工具、SD 动画/字体素材和说明一起保留。

- 副板：9 键 + GPIO34 电位器，I²C 地址0x42、协议1、8字节帧、CRC-8/SMBUS。保留冷启动恢复和失联重连。
- 主板：三盏灯控制、触摸、狗狗 SD 动画、本地时间、天气、每天07:00娱乐热搜/热播剧、五分钟息屏。
- 页面顺序：灯控 → 天气 → HOT&TV → 9KEY；9KEY 永远最后。
- HOT&TV 页按 K3 依当前分类顺序播标题1→2→3→4→5→1；支持 SD 语音缓存，切分类/离页重置顺序。Speaker 插座 GPIO26 DAC；最终音频转换调整后，用户确认声音清楚、杂音可接受。
- 粉色/橘色及已确认息屏唤醒行为保持源码原样。今天只同步，没有重新刷入或宣称重做实板测试。
- `FeitSecrets.h` 含 Wi-Fi/涂鸦凭据，仅保留本地；仓库提供 `FeitSecrets.example.h`。不要上传旧 merged 固件，它也含凭据。

## 编译、配置与回退

RLCD 五页版：ESP-IDF5.5.1 + Arduino3.3.3 component，OPI PSRAM80MHz。本机 `build.ps1` 仍保留已验证工具链绝对路径；`firmware/CMakeLists.txt` 引用桌面四页版 deps 以及 Arduino 安装的 U8g2、ArduinoJson。迁移电脑需修改这些路径，不能声称任意机器直接一键构建。完整最终 sdkconfig 另存 `sdkconfig.verified`，重建先复制为 sdkconfig；尤其保留 `CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM=y`。

本地私密配置在桌面 `RLCD-GPT-本地配置.json`、`firmware/LocalConfig.h`、`firmware/GatewayConfig.h`；不要在聊天或 GitHub 输出密钥。`configure.py` 生成网关头文件；首次重建仍需本地建立 LocalConfig.h。公开示例不含有效凭据。

RLCD 仅刷 app 分区 **0x10000**，不 erase flash、不动 NVS/otadata；应用3MiB。最后布局版编译大小2128784字节，刷入 hash verified。备份本地 `private-backup/before-battery-app.bin` 含凭据，不公开。不要重跑旧 `prepare_project.py`/`prepare_firmware_ui.py` 覆盖后续手工修复。

九键：Arduino-ESP32 3.3.3，主板 `PartitionScheme=min_spiffs`，副板默认分区。Adafruit GFX/BusIO、ArduinoJson 按文档安装；主副板协议配套，但正常仅更改主板用途不必重刷副板。源码同步不是刷板授权。

## 同步范围和下次入口

GitHub 仓库沿用 `rhchengm-crypto/HiveRobot`。`SOURCE_SYNC_20261105.json` 记录本机源文件与发布副本 SHA-256；发布前检查实际本地密钥与常见 token 模式，只输出通过/失败与路径，不输出密钥。

包含当前代码、修改过的依赖源码/许可证、SD 展示素材、配置模板、编译脚本与交接。排除真实配置、录音、个人日志、缓存问答/新闻、旧备份、可执行工具链和含凭据的固件产物。

用户对彩色替代板有兴趣，并要求保留 **2.54mm GPIO 扩展**。已推荐微雪3.5B/3.5（板载单麦克风、ES8311、SD、RTC、电池接口、2.54排针）以及3.49长条屏（双麦克风、22针2.54焊孔、18650版本）。尚未购买/换板/移植，购买前需核查实际可用GPIO是否足够外置全部按键，不能直接套现有引脚。当前黑白RLCD所有功能继续保留。

下一次从桌面五页版继续，先读本文件和实际设备状态。原收尾后用户又授权新闻 K1 播报功能，当前进展以下方追加记录为准。


## 后续追加：新闻页 K1 播报（2026-11-05）

原收尾后，用户又授权测试 RLCD 喇叭，并要求 K1 依次播新闻，最后一条后回第一条。当前实板已升级为新闻流式播报版，电池图标叠加百分比及原五页功能继续保留。

- 外置四键现在按物理左右顺序编号：K1=GPIO3、K2=GPIO2、K3=GPIO17、K4=GPIO1。只有新闻页 K1 增加播报；BOOT/KEY/RX 保持原功能。
- K1 首次从第1条开始，每次下一条，到实际最后一条后回第1条。播放中再按会切换下一条；快速多按取最后选择，不叠加多个声音。重启或新闻内容变化后重新从第1条。
- 读取当前新闻标题与摘要，不重新生成新闻。使用 `qwen3-tts-flash` / Cherry / Chinese，经原有直连Wi-Fi。新闻/Qwen/GPT回答模型均未改动。
- 未缓存条目通过经过证书验证的 ESP-IDF HTTP/SSE 接收，独立任务播放；完整音频存 `/rlcd/speech/`，同一条再次播放读 SD。缓存用8.3短文件名与完整SHA-256身份校验；不格式化卡、不存额外录音。
- ES8311 输出GPIO8、功放GPIO46；与ES7210共用MCLK16/BCLK9/WS45。总线保持16kHz双声道，24kHz单声道服务音频重采样后播放。播放前暂停麦克风codec，结束或取消后恢复24dB输入增益并关闭功放。
- 首版整条WAV生成下载用户确认有声，但等待太久。Arduino HTTP/SSE测试首段约3.2秒，但未完整收到stop，不能回退为成功版。当前改用ESP-IDF原生HTTP，已收到stop且完整播放。
- 5760字节接收窗口时，原生流式437804字节音频任务耗时35995ms；增大TCP接收窗口为32768字节、接收邮箱24项，发送缓冲不变。未缓存第6条实测首段2438ms、完整453164字节音频、任务14389ms并保存SD；缓存读取616～642ms。是本机单次测试，不是所有网络下的保证。
- 自动验证：真实命令顺序7→8→9→10→1，回到第1条缓存播放成功；播报后和休眠唤醒后均读取32000帧麦克风样本成功；休眠result=0、SD ready=1、电池ADC有效。
- 32KB窗口后GPT原生VPN重新连接ready=1，16KB上传诊断body_received=1、1396ms，退出后active=0。上传诊断用无效音频格式，在网关前置校验拒绝，不调用付费语音识别。
- 上游codec库在格式重开时会打印重复disable未启用通道的警告；实测open/write/输入恢复与采样均通过。不要把该条日志直接等同I2C离线。暂未修改该第三方库的启停语义。
- 用户已确认提示音与新闻有声；最终提速版用户已确认“较快出声，中文连续，顺序正确”；实体K1第1/2条通过。测试时已经验证并恢复过第1条起始游标，后续继续依当前按键次数循环。

源文件：`firmware/NewsSpeech.cpp/.h`、`PcmWave.h`、`Microphone.cpp/.h`、`ExternalKeys.h`、`main.cpp`、CMake与sdkconfig。详情 `NEWS_SPEECH.md`；日志摘要 `NEWS_SPEECH_VALIDATION.json`。诊断 `speech-info / speech-next / speech-stop / speaker-test`。本轮没有刷COM4灯控板。


## 2026-11-06：RX 轻睡眠恢复与 9 键对比

工作目录现为 C:/Users/rhche/Desktop/esp32/rlcd-gpt-tailscale；新 CAD 存 Desktop/3D print，ESP32 工程存 Desktop/esp32。用户澄清短时休眠可唤醒，整夜后失败；未认定根因。稳定性修正版已刷入：关闭轻睡眠中的 CPU 域断电，检查 GPIO44 唤醒配置，先完整初始化并重画 ST7305，再恢复 USB/麦克风/Wi-Fi。其他功能保留。

实际两分钟定时休眠 119999ms、恢复 310ms；三次 RX 唤醒用户确认均正常，计数 4、GPIO 原因 7、恢复 311ms。唤醒后实际麦克风读取 32000 帧、SD、Wi-Fi 通过。整夜休眠尚未验证，不能宣称整夜问题已解决。休眠电流可能略增，增量未实测。详细记录：rlcd-gpt-tailscale/SLEEP_RECOVERY.md 与 SLEEP_RECOVERY_VALIDATION.json。

9 键灯控只关背光并暂停动画，主控、按键扫描和网络继续运行；RLCD 使用 esp_light_sleep_start，二者恢复路径不同。没有更改或刷写 9 键板，也没有把 RLCD 改为息屏待机。build.ps1 默认构建目录改为 build-sleep-recovery，原 build 保留；私有配置、固件二进制、用户录音不发布。


## 2026-11-06：天气页假死与 VPN 退出修正

实板串口正常、天气 valid=1 verified=1，第 4 页却无法操作；GPT state=3 Failed、active=1 sending=0。根因为退出超时后管理任务遗弃保留实例，stopping() 永久为真并全局锁定导航。已修正为可取消的连接等待、保留所有权重试安全清理、退出意图保持至完成、收尾时允许非 GPT 页面切换；没有强删任务或提前释放资源。

最终版已编译并 app-only 刷入、哈希校验通过。立即退出、连接 3 秒后退出、连接成功后退出、同批指令离开并马上重新进入，共四项真实生命周期测试通过；每次天气→新闻→转文字均可切换，最终 VPN state=0 active=0。GPT 网关 16KB 上传通过；随后轻睡眠 2999ms、恢复 302ms，麦克风实读 32000 帧、SD、Wi-Fi 均正常。实体 KEY 整圈切换待用户确认；整夜休眠仍待验证。详见 rlcd-gpt-tailscale/VPN_EXIT_FIX.md 和 VPN_EXIT_VALIDATION.json。
