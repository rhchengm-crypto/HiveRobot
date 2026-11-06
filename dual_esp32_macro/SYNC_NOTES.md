# 最新灯控源码快照 — 2026-11-05

本目录源自桌面 dual_esp32_macro，最新功能修订为 2026-10-25 HOT&TV K3 中文播报版。保留三灯、触摸、狗狗SD动画、时间、天气、娱乐榜单、9KEY最后一页和五分钟息屏。旧 README 中待验证文字属于历史记录，最终已知状态见仓库根交接与 SPEECH.md；本次没有重刷或重做硬件验证。

复制 macro_main/FeitSecrets.example.h 为 FeitSecrets.h 并仅在本机填写真实配置。SD 素材在 sdcard/；这里只发布字体和展示动画，不含录音/用户缓存。源码、工具、素材与许可证同步；带密钥的 firmware/、build/ 和旧备份不上传。

主板 Arduino-ESP32 3.3.3 + min_spiffs，副板默认分区，其他依赖见 THIRD_PARTY.md。不要把 RLCD 的 COM19/ESP32-S3 构建配置用于此 ESP32 双板项目。
