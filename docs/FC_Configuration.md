# FC configuration
##### The FC used is a T-Motor F722, but anything should work.  
##### BF Version is: 4.2.6
------------
### Ports
![](https://i.imgur.com/1Gxg4fi.png)
------------
### Configuration
![](https://i.imgur.com/e0QRIBP.png)
```
feature -RX_PARALLEL_PWM
feature -AIRMODE
feature RX_SERIAL
feature MOTOR_STOP
feature TELEMETRY
```
------------
### PIDs
![](https://i.imgur.com/O0fiI0o.png)
```
set anti_gravity_gain = 1000
set p_pitch = 10
set i_pitch = 0
set d_pitch = 0
set f_pitch = 0
set p_roll = 0
set i_roll = 0
set d_roll = 0
set f_roll = 0
set p_yaw = 10
set i_yaw = 0
set f_yaw = 0
set d_min_roll = 0
set d_min_pitch = 0
```
------------
### Rates
![](https://i.imgur.com/J0mhFsz.png)
```
set thr_mid = 0
set thr_expo = 80
set roll_rc_rate = 200
set pitch_rc_rate = 200
set yaw_rc_rate = 200
set roll_srate = 0
set pitch_srate = 0
set yaw_srate = 0
set tpa_breakpoint = 1650
```
------------
### Receiver
F.Port 2.4GHz RX.
Channel Map is AETR1234!
 - Throttle left
------------
### Other CLI configs
```
set runaway_takeoff_prevention = OFF 
```
