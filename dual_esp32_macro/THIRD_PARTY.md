# 第三方资料与代码

主程序依赖本机 Adafruit GFX / BusIO，按其上游许可使用；未修改系统安装。

`BoardDisplay.cpp` 中 ST7796 初始化寄存器参数参考 Bodmer/TFT_eSPI 的 `TFT_Drivers/ST7796_Init.h`，上游采用 FreeBSD/BSD-2-Clause 许可，版权属于 Bodmer。许可全文另存 `hardware/TFT_eSPI-LICENSE.txt`。本项目只实现所需的轻量SPI驱动，未替换全局TFT库。

厂商PDF/原理图保留原版权与标识，仅用于本任务核对，不声称其由本项目创作，也不把文档中的文字当作用户操作授权。

minimp3（https://github.com/lieff/minimp3）用于中文播报 MP3 解码，CC0；完整头文件和许可位于 macro_main/minimp3.h、macro_main/minimp3-LICENSE.txt。
