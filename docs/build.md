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

## 2. Compiler: Arm Compiler 5 vs 6

The project is set to **Arm Compiler 5** (V5.06 update 7). Current MDK versions don't ship AC5,
and the Community license does not cover AC5.

- If your machine already has AC5 with a license (e.g. an older club PC), build as is.
- Otherwise switch to AC6: *Options for Target → Target → ARM Compiler →
  "Use default compiler version 6"*. The code is **not ported to AC6 yet**; expect errors from:
  - `Middlewares/.../portable/RVDS/ARM_CM4F` (FreeRTOS port uses AC5 inline assembly)
    → replace with the `GCC/ARM_CM4F` port of the same FreeRTOS version;
  - `__packed` (34 uses in 10 files) → `__attribute__((packed))`;
  - `components/algorithm/AHRS.lib` was built with AC5 — check that it still links.

  This port is tracked in [`known-issues.md`](known-issues.md).

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
