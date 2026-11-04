# RLCD 千问问答、上海话转普通话与每日新闻

微雪 ESP32-S3-RLCD-4.2（ESP32-S3 N16R8，400×300 横屏，ES7210 双麦克风）。
本目录同步桌面工程的最新源码，最后功能修复于 2026-11-02，同步日期 2026-11-03。

## 操作

长按 BOOT 约 1 秒切换三页，短按 BOOT 翻当前结果页。

1. 千问问答：KEY 开始录音，再按 KEY 提交；识别后调用千问回答，保留联网搜索。
2. 上海话转普通话：KEY 开始连续听写，等待“正在听”后讲话；停顿约 1 秒确认一句，转为标准普通话追加显示，只翻译、不回答。再按 KEY 停止。静音不会自动结束整个会话。
3. 每日社会新闻：设备开机联网且空闲时，每天 America/Los_Angeles 当地早上 7 点更新至多 10 条，自动处理夏令时，错过后补更。新闻页 KEY 可手动刷新，结果缓存于 NVS，失败保留旧缓存。是联网整理的热点精选，并非官方热度榜。

白底黑字、加粗显示。实时听写使用 qwen3-asr-flash-realtime，16 kHz 单声道 Opus 12 kbps VBR/DTX、40 ms 帧，最多 6 个 Ogg 页合包，PSRAM 队列约 10 秒。持续积压或网络错误仍会停止并提示，不自动重放；下次 KEY 开始新会话。上海话最终句子经 qwen-flash 转为标准普通话。
TLS 验证根证书及域名；包含 11 月 2 日的根证书、页面分离、实时连接及上传积压修复。

## 配置与编译

1. 安装 Arduino IDE / Arduino CLI、Arduino-ESP32 **3.3.3**，并安装 U8g2、ArduinoJson。
2. 在本目录终端运行 `python configure.py`，本地填写 Wi-Fi、百炼 API Key 和地域；或复制 `rlcd_qwen_asr/LocalConfig.example.h` 为 `LocalConfig.h` 后填写。
3. Windows 运行 `powershell -ExecutionPolicy Bypass -File build.ps1`，只编译、不刷入。

FQBN：`esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=cdc`。
`build.ps1` 默认使用 Windows Arduino IDE 自带 CLI；其他系统可用自己的 CLI 和同一 FQBN，并指定 `--libraries deps`。
刷板前核对目标 USB 设备和端口，避免刷到灯控板。

`deps/` 包含 WebSockets 2.7.2 和 RlcdOpus（含 ESP32-S3 预编译编码库）；显示驱动及 esp_codec_dev 在草图目录内。
该版本依赖上述核心及库接口，不承诺未验证版本兼容。

## 隐私与同步范围

仓库不包含 `LocalConfig.h`、Wi-Fi 密码、API Key、编译固件、录音、新闻缓存、个人日志或旧备份。
源码与依赖按本机文件 SHA-256 校验，记录在 `SOURCE_MANIFEST.json`。原桌面源码和私密配置保持原样。
语音会上传到所配置的阿里云服务；编译固件会含本地密钥，勿公开。
本次为源码同步，不重新刷板或执行云端请求。已有实板反馈：问答正常，连续转文字正常，不再出现上传跟不上录音。

## 第三方来源

- Waveshare：ESP32-S3-RLCD-4.2 示例，参考提交 `d8bf7813d3b53251577ff03c03596ad6ad3acae0`，保留 WAVESHARE-LICENSE 与 codec LICENSE。
- WebSockets：保留 deps/WebSockets 内原许可证。
- RlcdOpus：Espressif esp-adf-libs 固定提交 `029af747d7f8bddd4b774af58031f036d9bc2c3c`，原始哈希见 deps/RlcdOpus/ORIGIN.json；保留原许可证，仅适用于 Espressif 芯片。
- U8g2、ArduinoJson：由本地 Arduino 库管理器安装，遵循各自许可证。
