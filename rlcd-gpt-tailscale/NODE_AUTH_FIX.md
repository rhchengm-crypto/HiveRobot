# 2026-11-07：Tailscale 已入网设备不再重复使用 Auth Key

用户已在 Tailscale 为树莓派和 RLCD 关闭设备 Key Expiry，OpenAI Key 设置 Never。Auth Key 的到期与设备身份密钥不同；已注册设备原则上继续授权，但客户端若仍在 RegisterRequest 中提交过期 Auth Key，存在被拒绝的已报告问题。原本地 ml_coord.c 在每次注册中都附带 AuthKey；机器/节点密钥原已持久化 NVS。

参考：
- Tailscale 官方：https://tailscale.com/docs/features/access-control/auth-keys
- 已报告问题：https://github.com/tailscale/tailscale/issues/19501
- 采用 MicroLink 的另一项目的针对性补丁（不是 CamM2325 官方发布）：https://github.com/polymathrobotics/protective-stop/pull/178

实现：
- 所有新连接先发 node-only RegisterRequest，不带 Auth 字段。旧版没有 enrollment 标记的安装也如此，避免升级迁移时重发旧密钥。
- 服务端 MachineAuthorized=true 后将 enrolled 标记写入 microlink NVS 命名空间；已有机器、WireGuard、DISCO 密钥保持原样。新建身份时标记清零；原有 factory-reset 的命名空间清除也会清除此标记。
- 仅当服务端明确要求授权，且尚未完成入网或明确 NodeKeyExpired，才允许在下一次连接中提交 Auth Key。每个实例至多提交一次；如果入网密钥被服务器消费但响应丢失，后续又回到 node-only 探测。
- 仅有服务端 Error 字段、没有明确授权/到期标记时，不触发自动使用 Auth Key。已入网身份被撤销/删除，不会仅因普通未授权响应就自动用旧密钥重新加入。
- 明确拒绝后管理器可停止并安全清理，屏幕显示授权失败，保留 BOOT 重试能力。日志 ML_AUTH 仅记录是否携带 Auth Key、入网状态和持久化结果，不记录密钥或登录 URL。

验证不撤销 Auth Key、不清空 NVS、不变更后台身份、不改树莓派和 OpenAI Key；直接核对实际注册请求构造分支省略 AuthKey 且真实网关连通。并非声称已经等过90天或真实撤销过密钥。首次全新设备入网的 fallback 未在生产主板上通过清除身份来测试，避免破坏已授权设备。

预期：在当前设备身份保留、服务端设备密钥不失效的情况下，正常重连不再依赖原 Auth Key 的有效期。更换主板、清空身份、设备从后台删除或将来重新启用设备到期后，可能仍需要新的有效入网授权。

保留已验收的五页功能、问答模型、连续 Opus、分页缓存、按键、SD、天气新闻和轻睡眠策略。最终实板记录见 NODE_AUTH_VALIDATION.json。

实板通过：旧版本标记缺失时 auth_key_sent=0 enrolled=0，服务端授权确认后 persisted=1；重新进入 GPT 时 auth_key_sent=0 enrolled=1 且网关 ready。芯片硬复位（不擦NVS）后仍 enrolled=1、不发AuthKey，约5656ms ready；随后正常天气→新闻→转文字切换并回GPT，约6647ms ready。正常循环最长243ms。首轮主机脚本在天气刷新忙时提前发出切页而断言失败，后续重启脚本等待空闲后切页通过，未将首轮脚本描述为全绿。短睡眠2999ms、恢复448ms，实际32000帧麦克风采样、SD及Wi-Fi通过，唤醒后GPT重新ready。真人问答验收待用户反馈。

2026-11-07 真人问答验收：用户确认“正常显示回答”，保留当前版本。
