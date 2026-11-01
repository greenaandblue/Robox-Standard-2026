# Known issues

From a code review in Nov 2026 (firmware state = `Standard-Robot` commit `8757807`, June 2024).
Tick an item and add the commit when it is fixed. Locations are given by file and function,
because line numbers move.

Priority: **P0** = memory corruption, starved tasks, or a feature that clearly misbehaves;
**P1** = wrong in a match (rules, controls, aiming); **P2** = hidden trap or tech debt.

## P0

- [x] **CV UART: `memset` overflow.** `referee_usart_task()` cleared 200 bytes of a 50-byte
  buffer, zeroing other globals. *Fixed in "Fix continuous-fire flicker…".*
- [x] **CV UART: task never yields.** The loop in `referee_usart_task()` had no delay, so OLED
  (Low) and idle never ran. Added `osDelay(1)`. *Same commit.*
- [x] **CV UART: out-of-bounds read, no checksum.** `find_CpltPacket()` read up to 13 bytes past
  `rx_data` and never checked the checksum. Now bounded, checksum verified, newest packet
  taken. *Fixed in "CV uart: verify checksum…".*
- [x] **Continuous fire flickers.** `shoot_set_mode()` left `SHOOT_CONTINUE_BULLET` when
  *either* input was released (`||` instead of `&&`), so the trigger motor current toggled every
  1 ms. *Fixed in "Fix continuous-fire flicker…".*
- [x] **Door motor never stops.** `door_control()` restarted the move right after
  `door_stop()`; the servo stalled against its end stop. Now moves only on a switch change.
  *Same commit.*
- [ ] **IMU uplink frames are torn.** USART6 TX DMA is *circular*: it keeps re-sending
  `dma_buf` with the first frame's length while `sprintf` rewrites it. Fix: TX DMA normal mode
  in CubeMX, send a new frame only after the previous one finished, fixed rate, add a frame
  end. Best together with the binary IMU v2 in `comm/PROTOCOL.md`.
- [ ] **USART6 interrupt handler is the old referee one.** `USART6_IRQHandler` in
  `referee_usart_task.c` never calls `HAL_UART_IRQHandler`, so HAL UART callbacks only half
  work. Needed before any IDLE-line receive.
- [ ] **CV task stack is 128 words** and calls `sprintf("%f")`; stack overflow checking is
  off (`configCHECK_FOR_STACK_OVERFLOW`). Raise to 256+ or drop `%f`.

## P1

- [ ] **Referee system is not read at all.** USART6 was taken for CV, so `REFEREE_TOE` is always
  offline → no chassis power limit (`chassis_power_control()` uses the no-referee limit) and
  no barrel heat limit (`shoot_set_mode()`). Over power / over heat costs HP in a match.
  Plan: move CV to USART1 and give USART6 back to the referee parser (still in the file).
- [ ] **Mouse right does two things:** auto-aim (`gimbal_behavour_set()`) and friction wheels
  to `FRIC_UP` (`shoot_feedback_update()`). Bullet speed changes when aiming, may exceed the
  speed limit.
- [ ] **Right switch drives the door too** (`door_control()` uses `rc.s[0]`, same as gimbal and
  chassis mode). Changing mode opens/closes the door. Give the door a key.
- [ ] **Shift** is both spin (`chassis_set_contorl()`) and the servo "minus" key (`servo_task`).
- [ ] **Auto-aim yaw across ±π.** `gimbal_auto_control()` computes
  `absolute_angle − cv_yaw` without `rad_format`, and the Kalman filter then sweeps through the
  wrong side of the circle → short gimbal jerk. Wrap the target and reset the filter state on
  entering AUTO.
- [ ] **Auto-aim latency.** The CV packet is parsed only when the 50-byte DMA buffer is full
  (~3.5 packets). Use the UART IDLE interrupt.
- [ ] **CV side (report to CV team):** prediction is computed but the raw angle is sent;
  IMU radians are added to vision degrees; length byte says 13.

## P2

- [ ] `gimbal_PID_calc()`: `Deadband = -0.017f` is negative, so the dead band never applies.
  If someone makes it positive, the `else` branch zeroes `Iout` and pitch will sag.
- [ ] `set_cali_gimbal_hook()`: yaw offset hard-coded to `0x058d`; calibration is ignored.
  Must be updated by hand after the yaw motor is re-mounted.
- [ ] `gimbal_relative_angle_limit()`: limits commented out; `gimbal_cmd_to_chassis_stop()`
  disabled, so the chassis can move while the gimbal initializes or is limp.
- [ ] Spin speed is `rand() % 9` (0–8 rad/s, new value every 2 ms); comment says [0.5, 5].
- [ ] Kalman filter in `filter.c` runs at 1 kHz on a held value (gain ≈ 0.42), so it barely
  filters, and it uses `double` (software emulated on Cortex-M4F).
- [ ] `cv_Data` is written in an interrupt and read in a task without `volatile`.
- [ ] Chassis X/Y channels and WASD are swapped in `chassis_rc_to_control_vector()` instead of
  fixing the wheel mapping in `chassis_vector_to_mecanum_wheel_speed()`.
- [ ] Names: `UART7_CommandRoute()` handles USART6; `referee_usart_task.c` is really the CV task.
- [x] Port to Arm Compiler 6 (see `docs/build.md`). *Done in "Port to Arm Compiler 6".*
  Still to confirm on hardware: links with the AC5-built `AHRS.lib`, and the robot behaves the same.
