# Introduce
SuperPower 2019 formal project for SENTRY.

This project request **STMLib** from gitlab project **SP19_Utility**.

# Versions

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
- [BUG]Shoot heat can not be limited.

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
- [Import]Fix the referee bug : now can read game state correctly.
- Fix the sp_type.h, now can use ARM compiler v5.0.

## v0.2.6
- Add speed pid on gimbal but not change some yaw set yet.
- Delete the pid change before mode change.
- Turn off the friction when in auto mode.
- [BUG]Speedbalance pidgains may have influence on friction when friction init after chasis init.
- Add fram limit to 10.
- Change gimbal state refresh to 6ms.


## v0.2.5
- Update referee.
- Add USB mode for auto-aim.
- Update BSP.


