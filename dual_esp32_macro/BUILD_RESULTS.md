# 9KEY + POT 固定协议 V1 构建结果

2026-10-23，Arduino-ESP32 3.3.3，ESP32 Dev Module（4MB/默认分区）。

| 固件 | 结果 | 程序空间 | 全局RAM |
|---|---|---:|---:|
| 主板 macro_main | 通过 | 321315字节，24% | 22560字节，6% |
| 副板 macro_secondary | 通过 | 315443字节，24% | 22376字节，6% |

C++ static_assert编译器实际执行并通过：512种九键组合消抖/释放、独立弹跳、毫秒溢出、5120种边沿转换、CRC标准向量、16组协议往返与1024种单比特损坏、短帧/越界拒绝、电位器EMA单调性及端点收敛。

另外确认：主板总线恢复辅助函数及副板恢复条件与五键稳定备份一致；主板1秒等待、ACK后20ms等待、5秒总线恢复、副板300ms启动等待、3秒/15秒外设恢复逻辑保留。协议扩展后仍需实板重新验证。

用户已经确认的是此前五键v2版本5次冷启动成功、K1/K3及组合键正常。当前9+1版仅编译和逻辑测试通过，未自动烧录，不能声称九键/ADC/新帧在实板验证通过。

源文件与固件哈希见BUILD_RESULTS.json，固定副板记录见SECONDARY_V1.md，编译日志见build，固件二进制见firmware。

## K7 GPIO33补丁

副板K7从GPIO19改为GPIO33，重新编译通过（程序315443字节、全局RAM22376字节）。主板源码哈希与此前构建一致，主板无须重新编译/烧录。协议版本、帧长度、key7的bit6及冷启动恢复逻辑不变。GPIO33实际按键测试待用户完成；不把GPIO19未响应直接判为芯片损坏。

## 主板旋钮防抖（2026-10-24）

仅主板重新编译通过，程序321315字节、RAM22560字节。增加阈值32、0~100档滞回量化、端点吸附；额外编译期测试通过。副板全部源码和已归档二进制SHA256均核对未变，副板无需烧录。实板静止抖动改善程度尚待用户测试。


## 2026-10-24 Feit K4 integration

Main build passed on Arduino-ESP32 3.3.3: program 1110875/1310720 bytes; global RAM 48856/327680 bytes. Existing constexpr tests compile. Secondary source and all four firmware binary hashes unchanged; no rebuild needed. I2C/cold-start and PotControl checked byte-for-byte against quiet stable release. Not flashed; actual WiFi/Tuya/K4/touch concurrency requires hardware test.


## 2026-10-24 Feit K4 integration

Main build passed on Arduino-ESP32 3.3.3: program 1111671/1310720 bytes; global RAM 48872/327680 bytes. Existing constexpr tests compile. Secondary source and all four firmware binary hashes unchanged; no rebuild needed. I2C/cold-start and PotControl unchanged; LCD staged startup/reinitialization and diagnostics added. Root cause not yet confirmed. Not flashed; actual WiFi/Tuya/K4/touch concurrency requires hardware test.


## 2026-10-24 Feit K4 integration

Main build passed on Arduino-ESP32 3.3.3: program 1112175/1310720 bytes; global RAM 48888/327680 bytes. Existing constexpr tests compile. Secondary source and all four firmware binary hashes unchanged; no rebuild needed. I2C/cold-start, PotControl and LCD driver unchanged from LCD recovery release; brightness mailbox compile-time tests passed. Not flashed; knob/cloud brightness requires hardware test. Prior K4 and LCD recovery confirmed by user.


## 2026-10-24 Feit K4 master / K9 single mapping

Main build passed on Arduino-ESP32 3.3.3: program 1112291/1310720 bytes; global RAM 48888/327680 bytes. Existing constexpr tests compile. Secondary source and all four firmware binary hashes unchanged; no rebuild needed. I2C/cold-start, PotControl and LCD driver unchanged from LCD recovery release; brightness mailbox compile-time tests passed. Not flashed; K9 mapping requires hardware test. K4, knob and LCD recovery confirmed by user.


## 2026-10-24 Two bulbs / K8 / K4 master

Main build passed: 1113179/1310720 program bytes, 48904/327680 global RAM. Group-target constexpr tests and existing brightness/input tests pass. Secondary source and all firmware hashes unchanged; LCD driver, I2C/cold-start, knob filter/scheduler unchanged. New bulb MAC exactly matched via Tuya factory-info API and status/functions read successfully. No control POST sent during discovery. Not flashed; K8 and two-bulb master hardware tests pending. See TWO_BULBS.md.


## 2026-10-24 Two-bulb shared knob brightness

Main build passed: 1113495/1310720 program bytes, 48920/327680 global RAM. Group-target, per-lamp brightness pending isolation and existing brightness/input constexpr tests pass. Secondary source and all firmware hashes unchanged; LCD driver, I2C/cold-start, knob filter unchanged; two per-bulb scheduling slots with round-robin dispatch added. New bulb MAC exactly matched via Tuya factory-info API and status/functions read successfully. No control POST sent during discovery. Not flashed; shared brightness hardware test pending. Power controls previously confirmed by user. See TWO_BULB_BRIGHTNESS.md.


## 2026-10-24 Three-bulb K2/master/knob update

Main build passed: 1113883/1310720 program bytes, 48960/327680 global RAM. Group-target, per-lamp brightness pending isolation and existing brightness/input constexpr tests pass. Secondary source and all firmware hashes unchanged; LCD driver, I2C/cold-start, knob filter unchanged; three per-bulb scheduling slots with round-robin dispatch added. New bulb MAC exactly matched via Tuya factory-info API and status/functions read successfully. No control POST sent during discovery. Not flashed; three-bulb hardware test pending. Previous two-bulb version confirmed by user. See THREE_BULBS.md.


## 2026-10-24 Landscape two-page UI

Main build passed: 1119115/1310720 program bytes, 49032/327680 global RAM. Group-target, per-lamp brightness pending isolation and existing brightness/input constexpr tests pass. Secondary source and all firmware hashes unchanged; Cloud backend and key/pot callbacks unchanged. I2C/cold-start and knob filter unchanged; LCD adds hardware rotation1 and UI adds two pages with persistent touch calibration. New bulb MAC exactly matched via Tuya factory-info API and status/functions read successfully. No control POST sent during discovery. Not flashed; landscape/touch/page hardware test pending. Layout previews inspected and sample text/rect bounds checked. See LANDSCAPE_UI.md.


## 2026-10-24 Touch bulb cards

Main build passed: 1119551/1310720 program bytes, 49032/327680 global RAM. Group-target, per-lamp brightness pending isolation and existing brightness/input constexpr tests pass. Secondary source and all firmware hashes unchanged; Network/signing/TLS routines and key/pot callbacks unchanged; UI enqueues bulb toggles through a new bounds-checked touch entry. I2C/cold-start and knob filter unchanged; LCD adds hardware rotation1 and UI adds two pages with persistent touch calibration. New bulb MAC exactly matched via Tuya factory-info API and status/functions read successfully. No control POST sent during discovery. Not flashed; touch card power hardware test pending. Layout previews inspected and sample text/rect bounds checked. See TOUCH_BULB_CARDS.md.


## 2026-10-24 K1 purple / warm preset

Main build passed: 1124911/1310720 program bytes, 49056/327680 global RAM. Group-target, per-lamp brightness pending isolation and existing brightness/input constexpr tests pass. Secondary source and all firmware hashes unchanged; Network/signing/TLS unchanged. K1 all-lamp color preset added; brightness now respects per-lamp white/color mode. Touch UI, power keys, calibration and secondary unchanged. I2C/cold-start and knob filter unchanged; LCD adds hardware rotation1 and UI adds two pages with persistent touch calibration. New bulb MAC exactly matched via Tuya factory-info API and status/functions read successfully. No control POST sent during discovery. Not flashed; K1 color and HSV brightness hardware test pending. Layout previews inspected and sample text/rect bounds checked. See K1_COLOR.md.


## 2026-10-24 K1 partial-failure alternation fix

Main build passed: 1124915/1310720 program bytes, 49056/327680 global RAM. Group-target, per-lamp brightness pending isolation and existing brightness/input constexpr tests pass. Secondary source and all firmware hashes unchanged; Network/signing/TLS unchanged. K1 now advances its next preset on each processed event, independent of ACK success; failed lamp mask is logged. Color payloads and mode-aware brightness retained. Touch UI, power keys, calibration and secondary unchanged. I2C/cold-start and knob filter unchanged; LCD adds hardware rotation1 and UI adds two pages with persistent touch calibration. New bulb MAC exactly matched via Tuya factory-info API and status/functions read successfully. No control POST sent during discovery. Not flashed; K1 color and HSV brightness hardware test pending. Layout previews inspected and sample text/rect bounds checked. See K1_COLOR.md.


## 2026-10-24 K6 blue / warm all-lamp toggle

Main build passed: 1125123/1310720 program bytes, 49056/327680 global RAM. Group-target, per-lamp brightness pending isolation and existing brightness/input constexpr tests pass. Secondary source and all firmware hashes unchanged; Network/signing/TLS unchanged. K6 blue/warm toggle added independently of K1 purple/warm. ACK-independent alternation, power/brightness preservation and mode-aware dimming retained. Touch UI, power keys, calibration and secondary unchanged. I2C/cold-start and knob filter unchanged; LCD adds hardware rotation1 and UI adds two pages with persistent touch calibration. New bulb MAC exactly matched via Tuya factory-info API and status/functions read successfully. No control POST sent during discovery. Not flashed; K6 blue/warm hardware test pending; K1 working confirmed by user. Layout previews inspected and sample text/rect bounds checked. See K6_BLUE.md.


## 2026-10-24 K7/K5 page keys and quiet HEALTH log

Main build passed: 1125095/1310720 program bytes, 49056/327680 global RAM. Group-target, per-lamp brightness pending isolation and existing brightness/input constexpr tests pass. Secondary source and all firmware hashes unchanged; Network/signing/TLS unchanged. K7 previous/K5 next page added through shared touch/key page selection, bounded at edges. Main periodic HEALTH output removed; screen diagnostics and action/error logs retained. Cloud/color/dimming/secondary unchanged. Touch UI, power keys, calibration and secondary unchanged. I2C/cold-start and knob filter unchanged; LCD adds hardware rotation1 and UI adds two pages with persistent touch calibration. New bulb MAC exactly matched via Tuya factory-info API and status/functions read successfully. No control POST sent during discovery. Not flashed; page keys and quiet idle output hardware test pending; previous K1/K6 version accepted by user. Layout previews inspected and sample text/rect bounds checked. See PAGE_KEYS.md.


## 2026-10-24 Dog lamp cards

Main build passed: 1209663/1310720 program bytes, 49056/327680 global RAM. Group-target, per-lamp brightness pending isolation and existing brightness/input constexpr tests pass. Secondary source and all firmware hashes unchanged; Network/signing/TLS unchanged. Three dog lamp cards implemented with six indexed sprites, 83,200-byte PROGMEM asset data and 128-byte SPI transfer buffer. Card touch bounds enlarged to match. Backend, key/pot mappings, page keys, quiet HEALTH setting and secondary unchanged; calibration format retained. Touch UI, power keys, calibration and secondary unchanged. I2C/cold-start and knob filter unchanged; LCD adds hardware rotation1 and UI adds two pages with persistent touch calibration. New bulb MAC exactly matched via Tuya factory-info API and status/functions read successfully. No control POST sent during discovery. Not flashed; dog artwork/rendering/expanded card touches hardware test pending; previous page-key version accepted by user. Layout previews inspected and sample text/rect bounds checked. See DOG_UI.md.


## 2026-10-24 SD animation verified on COM4

Main 1253703/1310720 program bytes, 64184 static RAM. App flashed with hash verification; 166480-byte pack uploaded to /dogs/animation.dga, CRC edd0d04f, all 12 frames verified. Reset reload, continuing draws and LCD recovery passed; errors=0, maxReadUs=11148, maxDrawUs=13996. Control backend, key/pot and touch-handler source verified unchanged; secondary not flashed. Visual/touch/lamp operation still requires user observation. See ANIMATION.md.


## 2026-10-24 Weather 98074

Main 1284243/1310720 program, 64840 static RAM. App flashed and hash verified. Live three-day Open-Meteo forecast passed. Full navigation/cache retest completed: True. Weather uses idle shared network worker to avoid overlapping TLS allocations. Main control switch and 16 control/animation/hardware source files unchanged. See WEATHER.md.


## 2026-10-25 Pacific clock and pink/orange

Main 1285143/1310720 program, 64856 static RAM; app flashed and hash verified. NTP local clock, four DST-boundary samples, pages, dog pause/resume and LCD recovery passed. K1 HSV330/600 and K6 HSV30/1000 verified in compiled presets; no color POST sent. 19 animation/weather/hardware/control source files unchanged. See CLOCK_COLORS.md.


## Daily entertainment, 9KEY last

1299475/1310720 program bytes; 73064 static RAM. Main app flashed, partition unchanged. Actual 5+5, font CRC, schedule DST, four pages, animation CRC and reboot cache passed. See ENTERTAINMENT.md.


Pink refinement: HSV350/400; app flashed and presets verified. 1299475/1310720 program bytes, 73064 static RAM. All source files except ColorPreset.h unchanged.


Pink correction: HSV0/300; flashed and presets verified. 1299475/1310720 program bytes, 73064 static RAM. Only ColorPreset.h changed.


Selected swatch 01 cream pink #FFE4E1: HSV6/118; flashed and presets verified. 1299475/1310720 program bytes, 73064 static RAM. Only ColorPreset.h changed.


Five-minute display inactivity: 1300507/1310720 program bytes, 73088 static RAM. GPIO27 off/on, four pages and recovery while off passed. Full 300000ms serial capture interrupted by host suspend; user accepted existing behavior. See SCREEN_IDLE.md.

## HOT&TV K3 speech — 2026-10-25

主板 COM4 已写入并通过 esptool 哈希校验，采用 PartitionScheme=min_spiffs。中文标题播报顺序、缓存、分类重置、跳条/离页取消、唤醒及原有页面/动画/时钟的实板记录见 build/speech-test-summary.json；音质听感单独记录。副板未刷入。原有 NVS/触摸校准和 SD 素材保留。
