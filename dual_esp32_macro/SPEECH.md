# HOT&TV K3 中文播报

在 HOT&TV 页，按一次 K3 播报当前显示分类的一条标题，顺序为 1→2→3→4→5→1。热搜和热播剧都支持。播放中再按一次会停止当前条并进入下一条；切换分类或离开页面后，从第 1 条重新开始。标题更新后顺序也重置。

首次播放新标题需要 Wi-Fi，通过 Google Translate 中文语音接口取得 MP3，采用证书验证的 HTTPS；接口不可用时显示 VOICE UNAVAILABLE。该接口可能随服务端调整。已有语音保存在 SD 卡 /speech/，重复播放直接读卡，缓存命中时可离线播放。读取的是屏幕上的标题，不生成文章全文或剧情介绍。文件按完整播报文字哈希命名，旧缓存暂不自动删除。

板载 Speaker 插座使用 GPIO26 DAC 和原有 8002D 功放。minimp3 解码为单声道，插值至双倍采样率，采用一阶误差反馈减轻语音频段的 8 位量化噪声，经 DAC DMA 输出；软件音量为 40%，与灯光旋钮无关。8 个 2048 字节 DMA 缓冲，按 4096 音频采样成块写入，尾音等待 DMA 排空。解码缓冲分块申请以避免小内存板的堆碎片，播放任务按需建立、结束释放。串口记录解码耗时、读卡耗时及 DMA 停止次数，供音质排查；硬件底噪与喇叭音质仍需听音判断。

保留灯控、触摸、狗狗 SD 动画、当地时间和天气、每天 07:00 榜单更新、五分钟息屏。任意按键仍会唤醒并执行原功能。页面顺序：灯控→天气→HOT&TV→9KEY。灯控命令会停止正在播放的语音并优先处理；正在下载语音时，灯控命令仍需等待当前有超时限制的网络请求完成。

## 编译与分区

Arduino-ESP32 3.3.3，主板 FQBN：esp32:esp32:esp32:PartitionScheme=min_spiffs。主程序超过旧默认分区上限，因此主板已更新为 Minimal SPIFFS 分区，app0 仍从 0x10000 开始，容量 1966080 字节。NVS 0x9000/0x5000 和 otadata 0xe000/0x2000 保持原位置，触摸校准、天气和榜单缓存均保留。SD 卡素材不变。本次仅刷主板 COM4，副板保持原固件和默认分区。build.ps1 已分别选择主副板编译配置。

修改前源码保存在 backup-before-speech。原应用及原分区/NVS/otadata 备份保存在此次聊天 work/speech；该备份含本机配置，请保留私密。回退需要原应用与原分区布局配套，不能只换旧 merged 镜像或擦除全片。

## 验证与诊断

顺序及重置规则使用 constexpr 断言；实板检查包括中文 MP3 解码、SD 缓存、1～5 循环、热播剧切换、连续请求跳条、离页停止、息屏唤醒播报、狗狗动画、四页切换与时钟。详细结果见 build/speech-test-summary.json。实物 K3 已在串口捕获触发，用户确认提示音和中文可闻；最终音质以用户听音反馈为准。

串口命令（115200）：speech 查看状态，speak 在当前 HOT&TV 分类播下一条，speech stop 停止，speaker test 播短提示音，hotview hot / hotview tv 选择分类。speak 使用相同播报处理路径；人工按键和触摸行为需在实物上检查。

第三方解码器：https://github.com/lieff/minimp3 ，CC0，许可证见 macro_main/minimp3-LICENSE.txt。
DAC 驱动资料：https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/dac.html 。
