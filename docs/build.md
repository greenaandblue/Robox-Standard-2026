# Build and flash

µVision only runs on Windows. On a Mac, use a Windows virtual machine.

## 1. Install Keil MDK-Arm

1. Download **MDK-Arm** (5.43a or newer) from the Keil website and install it.
2. Activate the free **MDK Community** license (non-commercial use):
   *File → License Management → User-Based License → Activate / Deactivate*,
   then *License Server* = `https://mdk-preview.keil.arm.com` → *Query* →
   select *Keil MDK Community* → *Activate*.
3. Open `firmware/MDK-ARM/standard_robot.uvprojx`. If µVision asks for the device pack
   `Keil.STM32F4xx_DFP`, install it from the Pack Installer.

## 2. Compiler: Arm Compiler 6

The project builds with **Arm Compiler 6** (covered by the Community license). It was ported
from Arm Compiler 5 in Nov 2026:

- FreeRTOS port switched from `portable/RVDS/ARM_CM4F` (AC5 inline assembly) to
  `portable/GCC/ARM_CM4F` from the same FreeRTOS version (ST's V10.0.1 package);
- `__packed` replaced with `__attribute__((packed))` (same struct layout);
- `__CC_ARM` removed from the project defines, so CMSIS / `arm_math.h` take their AC6 paths.

After opening the project once, check *Options for Target → Target → ARM Compiler* says
**"Use default compiler version 6"** (select it if not). Leave *Asm → Assembler Option* on
**armasm (Arm Syntax)**: `startup_stm32f407xx.s` is written in Arm syntax.

AC5 is no longer supported (the GCC FreeRTOS port does not build with AC5).

## 3. Build

*Project → Build Target* (F7). Output goes to `firmware/MDK-ARM/standard_tpye_c/`
(ignored by git).

## 4. Flash

1. Install the driver for your debugger (ST-Link or J-Link).
2. In the VM software, pass the debugger's USB device through to Windows.
3. *Options for Target → Debug* → pick the debugger; *Flash → Download* (F8).

## Editing on the Mac

You can edit in VS Code on the Mac and build in the VM: put the repo in a folder shared with
the VM. `.vscode/c_cpp_properties.json` already has the include paths and defines from the
Keil project, so IntelliSense works.
