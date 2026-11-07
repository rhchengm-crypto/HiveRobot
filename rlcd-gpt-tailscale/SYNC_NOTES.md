# 2026-11-05 同步说明

当前实板运行本目录对应的五页完整版，最新加入新闻页 K1 顺序播报、流式播放与 SD 语音缓存；实测已确认出声较快、中文连续、顺序正确。README 开头的 experimental/production-baseline 描述是集成前的历史说明；实际验证与最新功能以仓库根 HANDOFF_20261105_RLCD_9KEY.md 为准。

保留桌面已验证代码字节，包括原生 VPN、SD、模型与 UI。Arduino3.3.3、修改后的 MicroLink/WireGuard 源码已随组件同步（排除上游 .git 和 Windows EXE 工具）。组件许可证保留。

重建需要 ESP-IDF5.5.1、Arduino3.3.3、U8g2、ArduinoJson 以及 ../rlcd-qwen-asr/deps 中的 WebSockets/RlcdOpus。现有 build.ps1 和 firmware/CMakeLists.txt 保留本机绝对路径，换电脑需修改。sdkconfig.verified 是本次成功编译的完整配置参考，先复制成 sdkconfig；sdkconfig.defaults 并未涵盖后续所有设置。

配置模板为 firmware/LocalConfig.example.h、firmware/GatewayConfig.example.h、LOCAL_CONFIG.example.json。后者按 configure.py 的约定在本机桌面保存为 RLCD-GPT-本地配置.json，不能将填入真值的文件提交。网关部署脚本还需本地安装 Paramiko；Linux/Pi 服务配置参考 gateway/。

build/、private-backup/、日志、用户历史、真实配置与固件二进制不发布。GitHub 同步没有再次刷板或调用语音 API。

新闻播报使用 ESP-IDF HTTP SSE，TCP 接收窗口 32768 字节，接收邮箱 24。首次未缓存出声约 2.44 秒，SD 缓存读取约 0.62 秒。NEWS_SPEECH.md 与 NEWS_SPEECH_VALIDATION.json 记录最终验证和用户验收；其他页面、麦克风、休眠与 GPT VPN 回归通过。

2026-11-06：RX 轻睡眠恢复修正版已刷入，短时测试通过，整夜待验证。源码工作目录迁移至 Desktop/esp32/rlcd-gpt-tailscale，build.ps1 默认使用 build-sleep-recovery；完整配置见 sdkconfig.verified，实测及 9 键对比见 SLEEP_RECOVERY.md。

天气页假死已修正 VPN 退出与页面锁定逻辑；四项生命周期、网关上传、休眠/麦克风回归通过，实体 KEY 用户已确认正常、不再卡住，整夜仍待验证。详见 VPN_EXIT_FIX.md。

2026-11-07：用户确认整夜休眠正常。随后实板复现转文字录音任务创建失败（frames=0）；改用 PSRAM 录音任务栈，固件已刷入并校验，四项实板采集/上传/停止回归通过（启动、重复、退出VPN、休眠唤醒）。详见 REALTIME_CAPTURE_FIX.md / REALTIME_CAPTURE_VALIDATION.json。真人识别结果待用户确认。

2026-11-07 真人验收：用户确认“正常出字，不再报录音任务失败”。


2026-11-07：GPT连接阶段/等待时间、BOOT可靠重试、VPN任务移至Core0、长回答精确分页缓存。六项实板回归通过，最长UI循环1741→200ms，参考分页与三页循环通过，短睡眠/麦克风/SD回归通过。真人验收以 GPT_RETRY_VALIDATION.json 为准。
