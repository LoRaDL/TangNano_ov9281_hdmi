# OV9281 寄存器配置整理

来源：`OV9281.pdf`，主要依据第 7 章 Register Tables。本文按功能模块整理寄存器地址范围、关键寄存器、默认值与用途，便于后续编写 SCCB/I2C 初始化表或调试脚本。

## 总体说明

- OV9281 通过 SCCB 访问寄存器。
- 设备从地址：写地址 `0xC0`，读地址 `0xC1`。
- 数据手册说明：寄存器表中的 enable/disable 位通常为 `ENABLE = 1`、`DISABLE = 0`。
- 大量地址为 reserved/debug/internal tuning，实际初始化时应优先使用厂家推荐序列或参考工程中已验证的配置。

## 寄存器功能分组总览

| 小节 | 功能 | 地址范围 | 主要用途 |
|---|---|---|---|
| 7.1 | System control | `0x0100-0x010A`, `0x3000-0x301F`, `0x303F` | 启停流、软件复位、芯片 ID、系统/MIPI/DVP 时钟复位与全局控制 |
| 7.2 | PLL control | `0x0300-0x0319` | PLL1/PLL2 分频、倍频、旁路、复位 |
| 7.3 | SCCB/group hold | `0x3100-0x3107`, `0x31FF-0x320F` | SCCB 调试、group hold、分组参数更新 |
| 7.4 | Manual AWB gain | `0x3400-0x3406` | 手动白平衡 R/G/B 增益 |
| 7.5 | Manual AEC/AGC | `0x3500-0x351D` | 手动曝光、模拟/数字增益、HDR 长短曝光/增益 |
| 7.6 | Analog control | `0x3600-0x3684` | 模拟相关控制，主要为保留/调试项 |
| 7.7 | Sensor control | `0x3700-0x37AF` | 传感器内部控制与 FIFO 控制 |
| 7.8 | Timing control | `0x3800-0x3835`, `0x3837` | 阵列裁剪、输出尺寸、HTS/VTS、镜像翻转、binning |
| 7.9 | PWM/strobe control | `0x3900-0x3904`, `0x3910-0x391D`, `0x3920-0x3933` | LED PWM、strobe 起止、极性与时序 |
| 7.10 | Low power mode | `0x4F00-0x4F0D`, `0x4F10-0x4F14` | PSV/自动休眠/低功耗相关时序和模拟参数 |
| 7.11 | BIST | `0x3E00-0x3E12` | 内建自测试地址、结果与状态 |
| 7.12 | OTP control | `0x3D80-0x3D87` | OTP 编程、加载、地址范围与时序 |
| 7.13 | BLC control | `0x4000-0x4017`, `0x4020-0x403F`, `0x4042-0x4049` | 黑电平校准、offset、阈值和比较参数 |
| 7.14 | Frame control | `0x4240-0x4244` | 帧开关数量、帧计数、帧事件 mask |
| 7.15 | Format control | `0x4300-0x4307`, `0x4311-0x4317`, `0x4320`, `0x4322-0x4329` | 数据范围、bit swap、嵌入数据、VSYNC、测试图案 |
| 7.16 | VFIFO control | `0x4600-0x4602` | VFIFO 读起点、帧复位、RAM bypass |
| 7.17 | DVP control | `0x4701-0x4709`, `0x470C`, `0x470F` | DVP VSYNC/HREF/PCLK 极性、VSYNC 输出和 bypass |
| 7.18 | MIPI top | `0x4800-0x4808`, `0x4810-0x483D`, `0x484A-0x484F` | MIPI lane、短包、LP/HS 时序、测试 pattern |
| 7.19 | ISP top | `0x5000-0x5018`, `0x5020-0x5024`, `0x5030-0x5035`, `0x5E00-0x5E2E` | ISP enable、窗口/缩放、测试图、读出状态 |
| 7.20 | Window control | `0x5A00-0x5A09`, `0x5A10-0x5A2F` | 手动输出窗口与 16-zone window 参数 |

## 关键寄存器配置说明

### 1. 启停流与复位

| 地址 | 名称 | 默认值 | R/W | 配置用途 |
|---|---|---:|---|---|
| `0x0100` | `SC_MODE_SELECT` | `0x00` | RW | bit0 选择模式：`0` software standby，`1` streaming |
| `0x0103` | `SC_SOFTWARE_RESET` | `0x00` | RW | bit0 软件复位：`1` 触发 reset |
| `0x0106` | `SC_FAST_STANDBY_CTRL` | `0x01` | RW | bit0 fast standby；`0` 等帧结束再进入，`1` 可截断帧进入 |
| `0x300A` | `SC_CHIP_ID_HIGH` | `0x92` | R | 芯片 ID 高字节 |
| `0x300B` | `SC_CHIP_ID_LOW` | `0x81` | R | 芯片 ID 低字节 |

建议初始化流程中常用：先写 `0x0103=0x01` 软复位，配置 PLL/timing/MIPI/DVP 等寄存器后，最后写 `0x0100=0x01` 开始 streaming。

### 2. SCCB 地址与接口

| 地址 | 名称 | 默认值 | R/W | 配置用途 |
|---|---|---:|---|---|
| `0x0107` | `CCI ADDRESS CONTROL 20` | `0x20` | RW | CCI slave 20 |
| `0x0108` | `CCI ADDRESS CONTROL 6C` | `0x6C` | RW | CCI slave 6C |
| `0x0109` | `CCI ADDRESS CONTROL C0` | `0xC0` | RW | CCI slave C0 |
| `0x010A` | `CCI ADDRESS CONTROL 7C` | `0x7C` | RW | CCI slave 7C |
| `0x31FF` | `SB_SWITCH` | `0x01` | RW | SCCB slave select，选择是否需要 XVCLK 的 SCCB slave |

### 3. PLL 与时钟

PLL 控制集中在 `0x0300-0x0319`。常见字段包括：

| 地址 | 名称 | 默认值 | 主要字段 |
|---|---|---:|---|
| `0x0300` | `PLL_CTRL_00` | `0x01` | `pll1_prediv` |
| `0x0301` | `PLL_CTRL_01` | `0x00` | `pll1_divp_h` |
| `0x0302` | `PLL_CTRL_02` | `0x32` | `pll1_divp_l` |
| `0x0303` | `PLL_CTRL_03` | `0x00` | `pll1_divm` |
| `0x0304` | `PLL_CTRL_04` | `0x03` | `pll1_div_mipi` |
| `0x030B-0x0315` | PLL2 controls | 多个默认值 | `pll2_prediv/divp/divs/cp/bypass/div_rst_sync` |
| `0x0318` | `PLL_CTRL_18` | `0x00` | `pll1_rst_o` |
| `0x0319` | `PLL_CTRL_19` | `0x00` | `pll2_rst_o` |

`0x300D-0x3024` 还包含系统时钟门控、复位、MIPI/DVP/ISP/BLC/AEC 等模块 clock/reset 控制。需要结合目标分辨率、帧率、MIPI lane 数和 XVCLK 频率计算 PLL。

### 4. MIPI / DVP 输出选择

#### System control 中的接口相关位

| 地址 | 名称 | 默认值 | 关键位 |
|---|---|---:|---|
| `0x3014` | `SC_MIPI_SC_CTRL0` | `0x04` | bit2 `mipi_en`，bit0 `lane_dis_op` 控制 lane disable 行为 |
| `0x3022` | `SC_MISC_CTRL` | `0x01` | bit[7:4] `mipi_bit_sel`：`0100` 8-bit，`0101` 10-bit，`0110` 12-bit；bit2/1 clock lane disable |
| `0x3039` | `SC_CTRL_39` | `0x32` | bit[7:5] MIPI lane 数：`000` one-lane，`001` two-lane；bit4 `mipi_en`；bit0 lane disable option |
| `0x303A` | `SC_CTRL_3A` | `0x00` | MIPI lane disable |

#### DVP 控制

| 地址 | 名称 | 默认值 | 配置用途 |
|---|---|---:|---|
| `0x4701` | `VSYNCOUT_SEL` | `0x00` | VSYNC 输出源选择 |
| `0x4702-0x4703` | `VSYNC_RISE_LNT` | `0x0002` | VSYNC 上升位置 line counter |
| `0x4704-0x4705` | `VSYNC_FALL_LNT` | `0x0006` | VSYNC 下降位置 line counter |
| `0x4706-0x4707` | `VSYNC_CHG_PCNT` | `0x0010` | VSYNC 变化像素位置 |
| `0x4708` | `POLARITY_CTRL` | `0x09` | bit7 DDR clock，bit5 VSYNC gate，bit4 HREF gate，bit2 HREF polarity，bit1 VSYNC polarity，bit0 PCLK polarity |
| `0x4709` | `BIT_TEST_ORDER` | `0x00` | bit[6:4] data bit swap |
| `0x470C` | `R_READ_CTRL` | `0x81` | read control |
| `0x470F` | `BYP_SEL` | `0x00` | bypass/href select |

#### MIPI top

| 地址 | 名称 | 默认值 | 配置用途 |
|---|---|---:|---|
| `0x4800` | `MIPI_CTRL00` | `0x04` | clock lane gate、line sync、PCLK 到 PHY 边沿选择、LPX 选择 |
| `0x4802` | `MIPI_CTRL02` | `0x00` | 自动或手动选择 hs/clk prepare、zero、trail、post、exit 等时序 |
| `0x4803` | `MIPI_CTRL03` | `0x10` | power mark、manual offset 等 |
| `0x4806` | `MIPI_CTRL06` | `0x00` | power mark、remote reset、suspend、low-bit-first |
| `0x4810-0x4811` | `MIPI_FCNT_MAX` | `0xFFFF` | frame sync short packet 最大帧计数 |
| `0x4813` | `MIPI_CTRL13` | `0x00` | virtual channel |
| `0x4814` | `MIPI_CTRL14` | `0x2A` | MIPI data type，默认 `0x2A` 常对应 RAW8 |
| `0x4818-0x4832` | MIPI timing min/UI | 多个默认值 | HS/CLK/LPX prepare、zero、trail、post、exit 等时序 |
| `0x4837` | `MIPI_PCLK_PERIOD` | `0x10` | PCLK2x 周期 |
| `0x484A-0x484C` | `MIPI_CTRL4A/B/C` | `0x3F/0x07/0x00` | sleep/clock start/SOF/HREF/PRBS/test-only 控制 |
| `0x484D-0x484F` | test pattern data | `0xB6/0x10/0x55` | MIPI data lane test pattern |

### 5. 图像窗口、分辨率与时序

#### Timing control

| 地址 | 名称 | 默认值 | 配置用途 |
|---|---|---:|---|
| `0x3800-0x3801` | `TIMING_X_ADDR_START` | `0x0000` | 阵列水平起点 |
| `0x3802-0x3803` | `TIMING_Y_ADDR_START` | `0x0000` | 阵列垂直起点 |
| `0x3804-0x3805` | `TIMING_X_ADDR_END` | `0x050F` | 阵列水平终点 |
| `0x3806-0x3807` | `TIMING_Y_ADDR_END` | `0x032F` | 阵列垂直终点 |
| `0x3808-0x3809` | `TIMING_X_OUTPUT_SIZE` | `0x0500` | ISP 水平输出宽度，默认 1280 |
| `0x380A-0x380B` | `TIMING_Y_OUTPUT_SIZE` | `0x0320` | ISP 垂直输出高度，默认 800 |
| `0x380C-0x380D` | `TIMING_HTS` | `0x02D8` | 总水平 timing size |
| `0x380E-0x380F` | `TIMING_VTS` | `0x038E` | 总垂直 timing size |
| `0x3810-0x3811` | `TIMING_ISP_X_WIN` | `0x0008` | ISP 水平 window offset |
| `0x3812-0x3813` | `TIMING_ISP_Y_WIN` | `0x0008` | ISP 垂直 window offset |
| `0x3814` | `TIMING_X_INC` | `0x11` | 奇/偶列增量 |
| `0x3815` | `TIMING_Y_INC` | `0x11` | 奇/偶行增量 |
| `0x3820` | `TIMING_FORMAT1` | `0x40` | 垂直 flip、vertical binning 等 |
| `0x3821` | `TIMING_FORMAT2` | `0x00` | 水平 mirror、horizontal binning 等 |
| `0x3837` | `DIGITAL_BINNING_CTRL` | `0x00` | horizontal digital binning enable/summation |

默认输出尺寸为 `1280 x 800`，由 `0x3808-0x380B = 0x0500 x 0x0320` 给出。裁剪、binning、镜像翻转通常围绕 `0x3800-0x3821` 调整。

#### Window control

| 地址范围 | 功能 |
|---|---|
| `0x5A00-0x5A07` | 手动输出窗口起点和宽高：`x_start/y_start/x_window/y_window` |
| `0x5A08` | `WINC_CTRL`，控制是否使用手动窗口、ROI、16-zone、valid signal 等 |
| `0x5A09` | `WINC_SEL`，zone/ROI manual source 选择 |
| `0x5A10-0x5A2F` | 16-window 的 X/Y start 与 window 宽高参数 |

### 6. 曝光、增益与白平衡

#### Manual AWB

| 地址 | 名称 | 默认值 | 配置用途 |
|---|---|---:|---|
| `0x3400-0x3401` | `AWB RED GAIN` | `0x0400` | AWB red gain `[11:0]` |
| `0x3402-0x3403` | `AWB GRN GAIN` | `0x0400` | AWB green gain `[11:0]` |
| `0x3404-0x3405` | `AWB BLU GAIN` | `0x0400` | AWB blue gain `[11:0]` |
| `0x3406` | `AWB MAN CTRL` | `0x01` | bit0 AWB manual control |

#### Manual AEC/AGC

| 地址 | 名称 | 默认值 | 配置用途 |
|---|---|---:|---|
| `0x3500-0x3502` | `LONG EXPO` | `0x000200` | 长曝光 `[19:0]`，低 4 bit 为小数部分 |
| `0x3503` | `AEC MANUAL` | `0x00` | 曝光/增益/数字增益手动控制与 delay option |
| `0x3505` | `GCVT OPTION` | `0x00` | 增益转换选项 |
| `0x3507` | `GAIN SHIFT` | `0x00` | 增益位移：无/左移 1/2/3 bit |
| `0x3508-0x3509` | `LONG GAIN` | `0x0080` | 长曝光模拟增益；格式受 `0x3503[2]` 影响 |
| `0x350A-0x350B` | `LONG DIGIGAIN` | `0x0400` | 长曝光数字增益 `[13:0]` |
| `0x350C-0x350D` | `SHORT GAIN` | `0x0080` | 短曝光模拟增益 |
| `0x350E-0x350F` | `SHORT DIGIGAIN` | `0x0400` | 短曝光数字增益 |
| `0x3510-0x3512` | `SHORT EXPO` | `0x000200` | 短曝光 `[19:0]`，低 4 bit 为小数部分 |
| `0x3513-0x3518` | SNR/fine gain readback | R | 长短曝光 SNR/fine gain 读回 |
| `0x3519-0x351D` | fine gain defaults/select | 多个默认值 | fine gain default/low/select |

### 7. 黑电平校准 BLC

BLC 寄存器主要位于：

- `0x4000-0x4017`：BLC 主控制、offset/trigger/filter/手动模式控制。
- `0x4020-0x402F`：BLC offset compare 参数。
- `0x4030-0x403F`：手动 offset 参数。
- `0x4042-0x4049`：BLC 输出、随机增益、阈值相关参数。

关键默认值示例：

| 地址 | 名称 | 默认值 | 配置用途 |
|---|---|---:|---|
| `0x4000` | `BLC_CTRL_00` | `0xCF` | BLC 权重/目标/offset compare/dither 等 enable |
| `0x4001` | `BLC_CTRL_01` | `0x20` | HDR/K coefficient/off-man/zero input 等控制 |
| `0x4003` | `BLC_CTRL_03` | `0x10` | `r_blc_lvl_target_o[7:0]` |
| `0x4005/0x4007` | `BLC_CTRL_05/07` | `0x02/0x02` | horizontal window offset/pad |
| `0x4010` | `BLC_CTRL_10` | `0x41` | gain/fmt/rst trigger、manual average/trigger、freeze 等 |
| `0x4011` | `BLC_CTRL_11` | `0x7F` | mf mode、gain/fmt change trigger 等 |
| `0x4042` | `BLC_CTRL_42` | `0x11` | format/gain/slope/manual cvdn/dn enable 等 |
| `0x4043` | `BLC_CTRL_43` | `0x40` | output select、random gain、dc/cvdn/bot blk enable 等 |

### 8. 测试图案与格式

| 地址 | 名称 | 默认值 | 配置用途 |
|---|---|---:|---|
| `0x4300` | `DATA_MAX H` | `0xFF` | 数据最大值高位 `[9:2]` |
| `0x4301` | `DATA_MIN H` | `0x00` | 数据最小值高位 `[9:2]` |
| `0x4302` | `CLIP L` | `0x0C` | data max/min 低位 |
| `0x4303` | `FORMAT CTRL3` | `0x00` | increment pattern、test bit shift 等 |
| `0x4304` | `FORMAT CTRL4` | `0x08` | data bit swap、test full window、bar pad |
| `0x4307` | `EMBED_CTRL` | `0x30` | embedded data enable/order/manual |
| `0x4311-0x4312` | `VSYNC_WIDTH_H/L` | `0x0400` | VSYNC width |
| `0x4313` | `VSYNC_CTRL` | `0x00` | VSYNC polarity/output/mode |
| `0x4314-0x4316` | `VSYNC_DELAY1/2/3` | `0x000100` | VSYNC trigger delay |
| `0x4317` | `MIPI/DVP MODE OPTION` | `0x00` | DVP enable |
| `0x4320` | `TST PATTERN CTRL` | `0x80` | pixel order、style swap、solid test pattern enable |
| `0x4322-0x4329` | `SOLID_Px_H/L` | `0x00` | solid test pattern P1/P2/P3/P4 值 |

ISP 侧还有 `0x5E00 PRE CTRL00` 测试图控制，默认 `0x00`，可选择 test pattern bar、random data、square、black image 等模式。

### 9. PWM / Strobe

| 地址范围 | 功能 |
|---|---|
| `0x3900-0x3904` | strobe row/cs 起始控制 |
| `0x3910-0x391D` | PWM divider、duty、LED duty cycle、平均值、斜率、极性 |
| `0x3920-0x392F` | strobe pattern、frame shift、divider value、step 等 |
| `0x3930-0x3933` | LED duty cycle / divider value 读回 |

关键默认值示例：

| 地址 | 名称 | 默认值 | 配置用途 |
|---|---|---:|---|
| `0x3910-0x3911` | `PWM_CTRL_10/11` | `0xFFFF` | `div_reg[15:0]` |
| `0x3912-0x3913` | `PWM_CTRL_12/13` | `0x0800` | duty_reg，0~65535 对应 0%~100% |
| `0x391D` | `PWM_CTRL_1D` | `0x10` | strobe frame VS select、PWM polarity |
| `0x3920` | `PWM_CTRL_20` | `0xA5` | strobe pattern |
| `0x392F` | `PWM_CTRL_2F` | `0x40` | strobe frame power/polarity/step 等 |

### 10. OTP 与 BIST

#### OTP

| 地址 | 名称 | 默认值 | 配置用途 |
|---|---|---:|---|
| `0x3D80` | `OTP PROGRAM CTRL` | `0x00` | program enable / start program |
| `0x3D81` | `OTP LOAD CTRL` | R | 写入 bit0 启动加载 |
| `0x3D82` | `OTP PROGRAM PULSE` | `0x40` | program strobe pulse |
| `0x3D83` | `OTP LOAD PULSE` | `0x03` | load strobe pulse |
| `0x3D84` | `OPT MODE CTRL` | `0x01` | program disable、auto/manual mode、memory select |
| `0x3D85-0x3D86` | `OTP START/END ADDR` | `0x00/0x1F` | manual mode 地址范围 |
| `0x3D87` | `OTP PS2CS` | `0x03` | PS to CSB timing |

#### BIST

| 地址范围 | 功能 |
|---|---|
| `0x3E00-0x3E03` | BIST 起止地址 |
| `0x3E04-0x3E07` | BIST 操作计数、特殊数据、SRAM select、control |
| `0x3E08-0x3E11` | BIST error/result/info/done 读回 |
| `0x3E12` | SRAM data read-only mode |

## 初始化表编写建议

1. 电源和时钟稳定后，执行软件复位：`0x0103 = 0x01`。
2. 配置 PLL：`0x0300-0x0319`，与 XVCLK、目标帧率、MIPI lane 和 bit depth 对齐。
3. 配置系统 clock/reset 与输出接口：`0x300D-0x303F`，特别是 MIPI/DVP enable、lane 数、bit depth。
4. 配置 timing/window：`0x3800-0x3837` 和必要时 `0x5A00-0x5A2F`。
5. 配置曝光/增益/BLC/format/test pattern 等：`0x3400`、`0x3500`、`0x4000`、`0x4300`、`0x5E00` 等。
6. 配置 MIPI/DVP 输出时序：MIPI 用 `0x4800-0x484F`，DVP 用 `0x4701-0x470F`。
7. 最后写 `0x0100 = 0x01` 进入 streaming。

## 最小可关注寄存器清单

如果只做 FPGA 采集调通，优先确认这些寄存器：

| 目标 | 寄存器 |
|---|---|
| 芯片是否在线 | `0x300A=0x92`, `0x300B=0x81` |
| 软复位 | `0x0103` |
| 开始/停止输出 | `0x0100` |
| 输出尺寸 | `0x3808-0x380B` |
| 行/帧 timing | `0x380C-0x380F` |
| 裁剪起止 | `0x3800-0x3807` |
| 镜像/翻转/binning | `0x3820`, `0x3821`, `0x3837` |
| MIPI/DVP 选择 | `0x3014`, `0x3022`, `0x3039`, `0x4317` |
| DVP 极性 | `0x4708` |
| MIPI data type / timing | `0x4814`, `0x4800-0x4808`, `0x4818-0x4837` |
| 曝光/增益 | `0x3500-0x3512` |
| 测试图 | `0x4320`, `0x5E00` |
