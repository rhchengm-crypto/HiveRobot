# 主板集成与以后只改主板

复制KeyBridge.h、KeyBridge.cpp、KeyLogic.h到现有UI工程，不要再复制参考程序的BoardDisplay或第二个setup/loop。主板使用I²C控制器1、GPIO21/22；保持原显示、触摸、宏引擎和其他应用。

```cpp
#include "KeyBridge.h"
KeyBridge keys;
void keyEvent(uint8_t key, bool down) {
  // key=1..9，按下/松开；绑定主板宏引擎。
}
void potEvent(uint16_t raw) {
  // raw=0..4095，已滤波；在主板做反向、标定、音量/亮度等映射。
}
// 原setup中：keys.begin(keyEvent, potEvent);
// 原loop中：keys.service();
```

keys.held()现为uint16_t，范围0~0x1ff；keys.pot()返回最近已交付的滤波ADC值；keys.online()判断是否在线。网络、蓝牙、长宏动作请入队，回调不要阻塞UI。I²C任务只收集快照，回调在调用service的UI任务执行。

旧代码keys.begin(keyEvent)仍可编译，但不会触发旋钮回调；必须使用新版KeyBridge/KeyLogic，不能保留一字节读取。固定8字节格式见PROTOCOL_V1.md。主板可只使用九键中的一部分，不需要要求副板改扫描。

参考主板的POT_REVERSED只影响主板映射；电位器外侧脚接反也不用重刷副板。副板今后不保存宏、用途或UI配置，保持现有输入硬件与协议即可长期复用。

旧版冷启动已由用户完成5次自动上线测试；本次协议扩展需要重新验证。64快照队列溢出会释放/同步按键并计数，长时间阻塞时丢失的中间事件无法还原。原应用如果使用I²C1，必须统一控制器所有权，不能重复初始化两个TwoWire(1)。

此工程是ST7796/XPT2046参考UI和输入桥，不凭空定义九个宏，也不默认发送USB/BLE键盘快捷键；宏执行保持在主板现有应用中。

主板防抖版：使用PotControl.h中的0~100档滞回量化器；参考主程序onPotChanged只有量化档位变化才执行后续宏动作。KeyBridge原始ADC通知门槛为32。保持副板协议V1不变。


## 2026-10-24：K4 Feit 控制

主板已集成 Feit direct 云端控制，K4 每次按下切换开/关。后台联网不阻塞 UI；副板不变。详见 [FEIT_K4.md](FEIT_K4.md)。


## 最新：480×320横屏与双页面

LIGHTS灯泡状态页与9 KEYS九键页，通过顶部标签触摸切换。首次四点触摸校准，之后自动保存。三灯控制/旋钮/副板不变。详见 [LANDSCAPE_UI.md](LANDSCAPE_UI.md)。此前竖屏布局说明已由此更新取代。
