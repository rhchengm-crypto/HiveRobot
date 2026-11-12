# RLCD GPT via native MicroLink and Raspberry Pi

Experimental integration branch, based on working near-vad-fast-endpoint-20261105 firmware.
Do not claim board validation before build and hardware tests complete.

The four existing functions retain their original direct Wi-Fi API paths. A fifth GPT tab
connects MicroLink to the existing tailnet and sends recorded audio to the private Pi gateway.
Only the gateway calls OpenAI. No exit-node/default route change is made on the board.
The Pi gateway explicitly ignores proxy environment variables and binds only its tailnet IPv4.

Local configuration: desktop `RLCD-GPT-本地配置.json`. Never commit or print this file.
`configure.py` writes firmware/GatewayConfig.h with VPN and gateway credentials, not OpenAI keys.
`deploy_gateway.py` installs the Python gateway and a system service on the Pi using SSH.
GatewayConfig.h and LocalConfig.h are private. Backups and binary firmware also contain credentials.

Gateway: authenticated HTTP over WireGuard, port 8765, protocol version 1.
`GET /v1/health` checks readiness; `POST /v1/ask` accepts 16kHz mono PCM16 WAV,
up to 20 seconds, and returns a concise Chinese GPT answer. API timeouts are bounded;
busy requests are rejected instead of paid twice, and no audio/text/key is written to service logs.
GPT uses single-turn Q&A with the Responses web_search tool. Weather/news/current-information
questions require a completed search; other questions use optional search. Source URLs
are included in the display answer and therefore saved in SD history.
Local tests use a fake backend; GATEWAY_VALIDATION.json and GATEWAY_AUDIO_VALIDATION.json
record real requests made on the Pi over Tailscale.

UI order: transcription, Qwen, GPT, weather, news. BOOT records/stops a GPT question;
KEY short press pages through results, long press changes tabs. RX retains sleep/setup.
VPN starts on GPT entry, stops on leaving or sleep, and reconnects on wake into GPT.
Weather/news background fetches wait until VPN exits to avoid concurrent TLS memory peaks.
They use their original direct paths when they resume; cached data remains visible.

Build: ESP-IDF 5.5.1 + Arduino 3.3.3 as a component, matching the original Arduino core.
Arduino component manifest is scoped to the selected libraries, rather than downloading unused
Zigbee, Matter, audio decoding or provisioning dependencies. Original manifest retained alongside.
Build.ps1 configures and builds. Partition offsets match the existing 3MB/16MB layout;
do not erase NVS or overwrite otadata during experimental flashing.

MicroLink upstream pinned commit 216da3300f0493b0860247d43f7af5ce29df63a5.
Local patches: task-exit acknowledgements before freeing resources, queued-packet cleanup,
fail-closed TCP VPN-IP binding and bounded TCP send retries, verified DERP TLS,
TCPIP-thread netif teardown, and epoch-based replay timestamps. Upstream licenses are retained.
The WireGuard /10 interface does not replace the Wi-Fi default route.
Native tailnet ACL behavior and repeated lifecycle stability require dedicated testing;
DISCO allowlists are not an application authorization mechanism. Gateway token is independently checked.

Current known risks: native MicroLink network-stack compatibility, concurrent TLS SRAM peak,
task shutdown, reconnects and repeated page visits. Production firmware remains in rlcd-qwen-asr.

SD storage uses the board's one-bit SDMMC wiring: CLK38, CMD21, D0=39 at 10MHz.
The firmware never formats the card. Startup verifies a small temporary write/read in its own
`/rlcd` folder. A missing or unsupported card leaves the screen and network features usable.
Daily UTF-8 JSONL files under `/rlcd/history/YYYYMMDD.txt` store transcription segments,
Qwen answers, and GPT questions/answers. Audio is not retained. A PSRAM-backed asynchronous
queue keeps SD writes outside the recording path; sleep waits for pending writes to finish.
`/rlcd/last-gpt.jsn` restores the last GPT answer after reboot, with a backup during replacement.
Serial diagnostics: `sd-info` reports readiness/writes/drops; `sd-test` writes a diagnostic record.
Do not remove the card while powered on. History files contain the user's spoken content.

WireGuard timestamps use synchronized epoch time rather than boot uptime, so persisted keys
can handshake after reboot. Desktop Tailscale preferences were restored to the original stopped state.

GPT uploads now encode 24kbps VBR Opus/Ogg on the ESP32. The Pi forwards the original compressed Ogg file directly
to OpenAI transcription, with no local decoding. Container sequence and duration are
validated before API calls. ffmpeg is used only to generate public test fixtures.
GPT recording/encoding uses a dedicated 40KiB PSRAM task stack and the matching
vTaskDeleteWithCaps cleanup. The existing Qwen task stack remains unchanged.

Decrypted packets enter lwIP through the netif input callback (tcpip_input), rather than
calling ip_input from the WireGuard worker. This fixes stalled uploads and concurrent TCP
netconn assertions. Serial gpt-net-test exercises a 16KiB rejected-format body without API calls.

KEY paging follows visible non-empty text in Ready/Result/Failure states, so GPT
answers restored from SD or retained across reconnects remain pageable. Recording
and recognition keep their original key handling.

Web search runs entirely on the Pi gateway; no board reflash is required to enable it.
The gateway provides synchronized UTC and America/Los_Angeles local time to avoid
UTC-date drift in weather questions. Missing weather locations are clarified rather
than inferred from the Pi exit IP. The original four direct-Wi-Fi functions are untouched.
GATEWAY_SEARCH_VALIDATION.json records a real dated weather search and its source.

Active GPT answer model: gpt-6.1-sol, with reasoning effort low and a 4096-token
combined reasoning/output budget. Visible responses retain the 3800-byte UTF-8 bound.
Transcription stays gpt-4o-mini-transcribe; native routing, search, paging and SD
storage protocols are unchanged. The gateway returns the actual OpenAI response
model as metadata, rather than relying on a model self-identification answer.
GPT61_PREFLIGHT_VALIDATION.json and GATEWAY_SEARCH_VALIDATION.json verify real
gpt-6.1-sol responses from the Pi. No firmware flash was needed for this migration.

Qwen answer and daily-news generation use qwen3.8-flash. The news request uses
the multimodal DashScope endpoint and text content blocks, retaining JSON, source
index, date and duplicate checks. Continuous Shanghai-to-Mandarin stays on
qwen-audio-3.1-asr-flash-message; the separate Qwen recording ASR is unchanged.
The Qwen and GPT cards show the answer model above the bottom controls, alongside
the page counter. GPT obtains the configured model from authenticated gateway health
and refreshes it from each actual answer response. No OpenAI credentials reach the board.

Battery display: GPIO4 / ADC1 channel 3 uses the onboard 3:1 divider and calibrated ADC readings. The top-right icon and ~percentage refresh from filtered voltage every 5 seconds. Percentage is an approximate single-cell 18650 voltage estimate, affected by load and charging; no charging-status signal is inferred. LOW appears at 3.50 V or lower. Invalid readings show --%. Serial command: battery-info.
Battery UI: the estimate is now centered inside the battery icon. XOR text remains visible across the filled and empty portions; LOW remains alongside the icon.

News speech: the physically leftmost K1 (GPIO3) cycles current news entries from 1 through the actual count, then wraps. Cached audio is stored on SD; uncached audio uses Qwen TTS SSE with separate network/player tasks. See NEWS_SPEECH.md for controls, codec sharing, and validation.

2026-11-06：RX 轻睡眠恢复稳定性修正、实测及与 9 键息屏方式的区别见 [SLEEP_RECOVERY.md](SLEEP_RECOVERY.md)。当前默认构建目录为 build-sleep-recovery。2026-11-07 用户确认“整夜休眠正常了”。

天气页无法操作的 VPN 退出修正、四项生命周期回归和验证范围见 [VPN_EXIT_FIX.md](VPN_EXIT_FIX.md)。

2026-11-07：连续转文字录音栈改用 PSRAM，诊断和实板测试见 REALTIME_CAPTURE_FIX.md / REALTIME_CAPTURE_VALIDATION.json。

2026-11-07：GPT 等待阶段/BOOT重试与长回答分页缓存优化，详见 GPT_CONNECTION_FIX.md / GPT_RETRY_VALIDATION.json。

2026-11-07：已入网节点重连不再发送原Tailscale Auth Key，迁移与重启实测见 NODE_AUTH_FIX.md / NODE_AUTH_VALIDATION.json。

2026-11-11 配网热点隔离、失败诊断及新 Wi-Fi 保存/重启实测：[WIFI_PORTAL_FIX.md](WIFI_PORTAL_FIX.md)。
