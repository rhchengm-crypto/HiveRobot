# 2026-11-11 配网热点与新网络连接

## 修改

- 配网先使用 AP-only，暂停旧 Wi-Fi 的后台重连。扫描或提交新网络时才启用 STA，连接期间禁止新扫描并停止已有扫描。
- 提交失败后回到 AP-only，手机继续使用热点重试；旧配置保留，退出配网时恢复。连接成功并获取 IP 后才保存新配置。
- 增加不含 SSID、密码或密钥的事件日志，记录认证错误编号、关联、IP 与热点客户端数量；界面区分网络找不到、认证/握手和 DHCP 超时。
- main.cpp 只更新构建标识；模型、录音、问答、界面与休眠代码未改。

## 实测

完整构建、应用区 0x10000 刷入与哈希校验通过。未清除 NVS、未格式化 SD。用户先前无法加入热点；修正版 AP-only 下有一个客户端，用户确认能连接热点并打开网页。

首次新网络提交：

~~~text
[WIFI_CONNECT] begin_status=6
[WIFI_CONNECT] failed=1 reason=2 associated=0 status=6 original_preserved=1
~~~

随后同版固件再次提交：

~~~text
[WIFI_EVENT] associated=1
[WIFI_EVENT] got_ip=1
[WIFI_SAVE] success=1
~~~

热点关闭后 connected=1。芯片硬重启后 stored=1、portal=0、connected=1、reason=0、associated=1、mode=1；证明新配置已保存并自动连接。用户确认“连接成功了”。详细记录见 WIFI_PORTAL_VALIDATION.json。

## 原因与证据边界

失败码 2 是 AUTH_EXPIRE，出现在关联完成前，可能是认证响应超时或路由器返回的断开原因；不能据此直接判定密码错误。失败和成功使用同一版固件，因此重试成功也不能证明单一的无线环境或路由器根因。未记录信号强度、安全模式、失败提交与成功提交的凭据对照或路由器日志，无法进一步区分。

begin_status=6 是异步连接启动时尚未连接的状态，两次提交均出现，并非最终失败原因。连接成功后的 WIFI_INFO reason 可能保留上一次断线原因，以 connected/associated 与 WIFI_SAVE 为准。

旧网络后台重连和扫描可能干扰配网，代码已隔离；热点恢复已实测，但尚无证据将最初所有失败都归因于同一问题。

参考：[乐鑫 ESP32-S3 连接场景](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/wifi-driver/station-scenarios.html)。
