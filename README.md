# Robox Standard 2026 — embedded

Firmware for the Penn State RoboX **Standard (infantry) robot**, maintained by the embedded
systems team, plus the serial link between the robot and the CV mini-PC.

- MCU: DJI RoboMaster Development Board C (STM32F407IGH6), FreeRTOS, STM32 HAL 1.7.6
- Toolchain: Keil MDK-Arm (µVision), see [`docs/build.md`](docs/build.md)
- Based on DJI's open-source `standard_robot` example; history imported from
  [`PennState-RoboX/Standard-Robot`](https://github.com/PennState-RoboX/Standard-Robot)
  (`git log --follow <file>` shows a file's commits from before the move into `firmware/`)

## Layout

```
firmware/              Keil project (open firmware/MDK-ARM/standard_robot.uvprojx)
  application/         robot logic: one file per FreeRTOS task (gimbal, chassis, shoot, INS, ...)
  bsp/boards/          board support: PWM, UART, SPI, LEDs, buzzer
  components/          PID, filters, IMU drivers (BMI088, IST8310), AHRS library
  Src/ Inc/            STM32CubeMX-generated init code (regenerate from standard_robot.ioc)
  Drivers/ Middlewares/  ST HAL, CMSIS, FreeRTOS, USB — third party, don't edit
comm/
  PROTOCOL.md          STM32 <-> mini-PC protocol (aim downlink, IMU uplink)
  tools/               fake_cv.py, imu_monitor.py — test the link without CV
docs/
  build.md             build and flash
  known-issues.md      bug list from the code review, with status
```

## What runs on the board

| Task | File | Priority | Job |
|---|---|---|---|
| INS | `INS_task.c` | Realtime | IMU attitude (yaw/pitch/roll) from BMI088 + IST8310 |
| Gimbal | `gimbal_task.c`, `gimbal_behaviour.c` | High | yaw/pitch cascaded PID, 1 ms; also runs the shooter (`shoot.c`) |
| Chassis | `chassis_task.c`, `chassis_behaviour.c` | AboveNormal | 4 mecanum wheels, follow gimbal, spin |
| CV link | `referee_usart_task.c` | Normal | USART6: aim packets in, IMU text out ([protocol](comm/PROTOCOL.md)) |
| Detect | `detect_task.c` | Normal | marks devices offline (motors, remote, CV link, ...) |
| Others | `calibrate`, `servo`, `led_flow`, `oled`, `voltage`, `usb` | Normal/Low | |

The remote control (DBUS, USART3) is handled in an interrupt in `remote_control.c`.

## Controls (current firmware)

**Right switch** (`rc.s[0]`): sets gimbal + chassis mode, and also drives the magazine door.

| Position | Gimbal | Chassis | Door |
|---|---|---|---|
| up | absolute angle (IMU) | follows gimbal | opens (on change) |
| mid | **auto-aim** (CV) | does not follow gimbal | closes (on change) |
| down | no force | no move | — |

**Left switch** (`rc.s[1]`): flick up = friction wheels on/off; hold down = fire.

**Keyboard / mouse**

| Input | Action |
|---|---|
| W A S D | move |
| Shift (hold) | spin (小陀螺), right switch up only — also the "minus" modifier for servo keys |
| Ctrl (hold) | swing offset (chassis turned 0.6 rad from gimbal), right switch up only |
| Q / E | friction wheels on / off (left switch mid) |
| mouse left (hold) | fire |
| mouse right (hold) | auto-aim **and** friction wheels to high speed |
| Z X C V | servo PWM up (with Shift: down) |

Known conflicts (mouse right, Shift, right switch) are listed in
[`docs/known-issues.md`](docs/known-issues.md).

## Rules for contributors

1. **Files are UTF-8.** Don't let your editor re-save them as GBK (that corrupted comments
   before). VS Code settings and `.editorconfig` in this repo already enforce UTF-8.
2. Build in Keil before pushing. Note in the PR whether you also tested on the robot.
3. If you change bytes on the serial link, update `comm/PROTOCOL.md` in the same PR and tell
   the CV team.
4. Don't edit `Drivers/` or `Middlewares/`. Change peripherals in CubeMX (`standard_robot.ioc`)
   and keep your code inside `/* USER CODE BEGIN */ ... /* USER CODE END */` in `Src/`.
