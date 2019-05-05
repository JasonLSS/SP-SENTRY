# Introduce
SuperPower 2019 formal project for SENTRY.

This project request **STMLib** from gitlab project **SP19_Utility**.

# Versions
## v0.2.7
- Modefied gimbal PID.
- Add frame lost delay.
-[Import]Fix the referee bug : now can read game state correctly.
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


