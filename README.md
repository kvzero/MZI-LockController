[English](#mz-digital-lock) | [中文](#mz-digital-lock-中文)

# MZ Digital Lock

This repository contains the STM32G4 firmware for a Mach-Zehnder digital lock controller. The controller reads the interferometer output (`Iout`) and laser reference (`Iref`), computes the normalized ratio `R = Iout / Iref`, and drives a high-voltage DAC/PZT actuator to keep the interferometer near the selected lock point.

The current firmware supports:

- offset acquisition for the photodiode channels
- fringe scan and contrast/target extraction
- soft lock capture around the selected `Rtarget`
- resonance identification with DDS injection and IQ accumulation
- long-running hard lock with contrast-aware error scaling, main-resonance notch filtering, and a shaped PI loop
- simple LCD/button pages for wait, acquire, lock, and fault states
- HV amplifier and low-contrast fault handling

## Software Flow

```text
main.c
  -> APP_Init()
  -> APP_Process()
  -> APP_RenderIfNeeded()
  -> APP_OnButton()
```

Top-level UI state is page-based:

```text
APP_PAGE_WAIT
  -> APP_PAGE_ACQUIRE
       -> offset acquisition
       -> scan
       -> soft lock capture
       -> resonance sweep
       -> 1 s resonance summary
  -> APP_PAGE_LOCK
       -> hard lock long-running display
  -> APP_PAGE_FAULT
```

Realtime ADC/DAC work is separated from the UI page loop:

```text
DMA IRQ
  -> PDADC_DMA_IRQHandler()
  -> APPRTLOOP_OnDone()
       -> APP_RTLOOP_MODE_SCAN: APPSCAN_OnSample()
       -> APP_RTLOOP_MODE_LOCK: APPLOCK_OnSample()
```

The lock backend uses this staged state machine:

```text
APPLOCK_STATE_RAMP_TO_CENTER
  -> APPLOCK_STATE_SWEEP_DOWN
  -> APPLOCK_STATE_SOFT
  -> APPLOCK_STATE_RESONANCE
  -> APPLOCK_STATE_HARD
```

## Main Modules

```text
Core/Src/main.c
  Hardware initialization and foreground application loop.

Libraries/App/app_core.c
  Top-level page dispatch, render scheduling, and fault switching.

Libraries/App/app_page_acquire.c
  Acquisition workflow: offset, scan, soft lock, resonance sweep, then lock page.

Libraries/App/app_page_lock.c
  Long-running lock display and hard-lock start.

Libraries/App/app_rtloop.c
  Realtime ADC sample dispatch and DAC write ownership for scan/lock modes.

Libraries/App/app_scan.c
  Fringe scan, ratio extrema, contrast, and target extraction.

Libraries/App/app_lock.c
  Soft capture, resonance sweep, hard-lock PI loop, notch, and error shaping.
```

## Hard Lock Path

```text
ADC sample
  -> offset correction
  -> R = Iout / Iref
  -> error = Rtarget - R
  -> polarity correction
  -> contrast normalization
  -> main resonance notch
  -> low-shelf error shaping
  -> PI output
  -> HVDAC raw write
```

The hard-lock filter is shaped so that low-frequency error keeps stronger control authority while higher-frequency mechanical modes are driven less aggressively. The main notch is centered from the measured resonance frequency.

## Build

The active embedded project is the Keil MDK project:

```text
Lock_Controller/MDK-ARM/Lock_Controller.uvprojx
```

Generated STM32Cube/HAL sources live under `Lock_Controller/Core` and `Lock_Controller/Drivers`; application code lives under `Lock_Controller/Libraries/App`.

---

# MZ Digital Lock 中文

本仓库是基于 STM32G4 的马赫-曾德尔干涉仪数字锁定控制器固件。系统读取干涉仪输出通道 `Iout` 和激光参考通道 `Iref`，计算归一化比值 `R = Iout / Iref`，再通过高压 DAC/PZT 执行器把干涉仪保持在目标锁定点附近。

当前固件支持：

- 光电探测通道 offset 采集
- 条纹扫描，以及衬比度/目标锁定点提取
- 围绕 `Rtarget` 的软锁捕获
- 基于 DDS 注入和 IQ 累加的谐振频率识别
- 用于长期运行的硬锁定：包含衬比度相关误差缩放、主谐振陷波器和整形 PI 环路
- 等待、采集、锁定、故障等 LCD/按键页面
- 高压放大器故障和低衬比度故障处理

## 软件流程

```text
main.c
  -> APP_Init()
  -> APP_Process()
  -> APP_RenderIfNeeded()
  -> APP_OnButton()
```

顶层 UI 由页面状态机管理：

```text
APP_PAGE_WAIT
  -> APP_PAGE_ACQUIRE
       -> offset 采集
       -> 条纹扫描
       -> 软锁捕获
       -> 谐振扫描
       -> 谐振结果显示 1 s
  -> APP_PAGE_LOCK
       -> 长期硬锁定显示
  -> APP_PAGE_FAULT
```

实时 ADC/DAC 路径和前台 UI 页面循环分离：

```text
DMA IRQ
  -> PDADC_DMA_IRQHandler()
  -> APPRTLOOP_OnDone()
       -> APP_RTLOOP_MODE_SCAN: APPSCAN_OnSample()
       -> APP_RTLOOP_MODE_LOCK: APPLOCK_OnSample()
```

锁定后端使用以下分阶段状态机：

```text
APPLOCK_STATE_RAMP_TO_CENTER
  -> APPLOCK_STATE_SWEEP_DOWN
  -> APPLOCK_STATE_SOFT
  -> APPLOCK_STATE_RESONANCE
  -> APPLOCK_STATE_HARD
```

## 主要模块

```text
Core/Src/main.c
  硬件初始化和前台应用主循环。

Libraries/App/app_core.c
  顶层页面分发、渲染调度和故障切换。

Libraries/App/app_page_acquire.c
  采集流程：offset、扫描、软锁、谐振扫描，然后进入锁定页面。

Libraries/App/app_page_lock.c
  长期锁定显示和硬锁定启动。

Libraries/App/app_rtloop.c
  实时 ADC 采样分发，以及 scan/lock 模式下的 DAC 写入归属。

Libraries/App/app_scan.c
  条纹扫描、比值极值、衬比度和目标锁定点提取。

Libraries/App/app_lock.c
  软锁捕获、谐振扫描、硬锁 PI 环路、陷波器和误差整形。
```

## 硬锁定路径

```text
ADC sample
  -> offset correction
  -> R = Iout / Iref
  -> error = Rtarget - R
  -> polarity correction
  -> contrast normalization
  -> main resonance notch
  -> low-shelf error shaping
  -> PI output
  -> HVDAC raw write
```

硬锁定滤波器的目标是让低频误差保留较强控制作用，同时减少对高频机械模态的驱动。主陷波器中心频率来自自动测得的谐振频率。

## 构建

当前嵌入式工程使用 Keil MDK：

```text
Lock_Controller/MDK-ARM/Lock_Controller.uvprojx
```

STM32Cube/HAL 生成代码位于 `Lock_Controller/Core` 和 `Lock_Controller/Drivers`，应用层代码位于 `Lock_Controller/Libraries/App`。
