## Copy file between machines using ssh
Refer to this [answer](https://unix.stackexchange.com/questions/106480/how-to-copy-files-from-one-machine-to-another-using-ssh) <br/>
To copy a file from `B` to `A` while logged into `B`: <br/>
`$ scp /path/to/file username@a:/path/to/destination` <br/>
To copy a file from B to A while logged into A: <br/>
`$ scp username@b:/path/to/file /path/to/destination` <br/>

## Maximum angular velocity
Spinning too fast may cause SLAM fail. By investigating `/cmd_vel`, max "turn speed" is around 0.8, but 0.6 is an appropriate value as the cap.

## Monitor power input
Refer to this [post](https://devtalk.nvidia.com/default/topic/1000830/jetson-tx2-ina226-power-monitor-with-i2c-interface-/) <br/>
`$ cat /sys/devices/3160000.i2c/i2c-0/0-0040/iio_device/`*`rail_name_*`*
substitute *`rail_name_*`* to following as you wish
```
VDD_SYS_GPU
VDD_SYS_SOC
VDD_4V0_WIFI
VDD_IN
VDD_SYS_CPU
VDD_SYS_DDR
VDD_MUX
VDD_5V0_IO_SYS
VDD_3V3_SYS
VDD_3V3_IO_SLP
VDD_1V8_IO
VDD_3V3_SYS_M2
```

