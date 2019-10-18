# Introduce

SuperPower 2019 formal project for SENTRY.

This project request **STMLib** from gitlab project **SP19_Utility**.

# Versions
## v2.3.0

- Renew all BSP.

## v2.2.0

- The latest version of 2019.
- Add graphic draw.
- Change PID.

## v2.1.0

- Renew IMU.(But not use)
- Change gimbal PID.
- Add gimbal_yaw_angle to communicate.

## v2.0.8

- Add new gimbal PID.
- reduce shoot spped.
- [BUG] Speed change limit have unexpected return.
- Change attacked judging condition.
- Add new boot music.

## v2.0.7

- [IMPORTANT] Complete double board connect.
- Add new PID.
- Change chasis escape mode logic.

## v2.0.5

- [IMPORTANT] Add double board connect.(test)

## v2.0.2

- Add light IO.
- Change gimbal PID.(test)

## v2.0.1

- Add pitch cruise.
- Add protect when pitch can't reach target value.
- Change chasis TOF Threshold.
- Updata new referee.
- CLose the STATIC_ATTACK_MODE.
- 

## v2.0.0

- Change the USB communication protocols.
- Add neural PID for test.
- Change chasis remote control.
- Add convertible distance threshould.

## v1.1.0

- Increase speed limit.
- Add protect of different speed.
- [IMPORTANT] Final version of the division.

## v1.0.4

- Add back protect of gimbal change.
- Change yaw and pitch limit,and yaw_cruise speed.
- Change SPEED_CHANGE_LIMIT.
- Add protect of chasis by angle.

## v1.0.3

- [Important] Fix the auto aim bug of double degree.
- Delete protect of gimbal change.
- Add gimbal keep model.

## v1.0.2

- Change chasis power heat control.
- Close speed balance.

## v1.0.1

- Add watchdog.
- Add pitch seperate.
- Fix the bug of middle looper.
- Change the start speed of friction.
- Change power limit.
- Change frame pitch max.

## v0.3.7

- Add static mode protect.
- Add motor stall protection for sensor offline.
- Add feed motor protect.
- Change tof address to L-160-3 and R-166-1.
- Change communicate rules with miniPC.

## v0.3.6

- Add protect of chasis sensor,which need to be test.
- Fix the gimbal switch delay bug.

## v0.3.5

- Add USB send.
- Delete pitch cruise.

## v0.3.4

- Modified shoot heat limit.

## v0.3.3

- [Important] Fix the BUG of PWM output.
- [Important] Fix the referee read.
- Change USB NVIC.

## v0.3.2

- Add USB.
- Modified frame struct.
- Modified gimbal PID with speed pid.
- [BUG] Shoot heat can not be limited.

## v0.3.1

- Add shoot way.
- First auto aim success.

## v0.3.0

- Modified gimbal PID.(use 120fps camera.)

## v0.2.9

- Modified gimbal PID.Now can use auto_aim(test).
- Add yaw seperate.
- Add autoMode shoot control,use ch0 to shoot.

## v0.2.8

- Add gimbal small and large PID for different frame value.

## v0.2.7

- Modified gimbal PID.
- Add frame lost delay.
- [Import] Fix the referee bug : now can read game state correctly.
- Fix the sp_type.h, now can use ARM compiler v5.0.

## v0.2.6

- Add speed pid on gimbal but not change some yaw set yet.
- Delete the pid change before mode change.
- Turn off the friction when in auto mode.
- [BUG] Speedbalance pidgains may have influence on friction when friction init after chasis init.
- Add fram limit to 10.
- Change gimbal state refresh to 6ms.

## v0.2.5

- Update referee.
- Add USB mode for auto-aim.
- Update BSP.
