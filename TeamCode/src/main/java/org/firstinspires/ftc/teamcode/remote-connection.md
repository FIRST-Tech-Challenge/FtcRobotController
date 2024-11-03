Why
---
To deploy and capture latest logcat output when robot is moving

Howto
---
- connect work computer to WIFI name: FTC-dvEo (password: password)
- make sure adb is available in terminal
- open terminal
- Run `adb kill-server&&adb start-server`. This command will restart the adb
- run `adb connect 192.168.43.1:5555`
    - some times if previous computer connected to the control hub and does not disconnect properly, you will not able to connect, restart the control hub (pull the power plug and reconnect) should fix it
    - you should see something like ```connected to 192.168.43.1:5555```
- verify the connection by run `adb devices`
- You should see output looks like
```
  List of devices attached                                                                                                                                                     ─╯
  192.168.43.1:5555       device
```
- verify the device in android studio
    - in device manager, under "Running devices" section, you should see a device with name `REV Robotics Control Hub 1.0` listed
    - Select that device and run the teamcode will deploy your code to the control hub remotely

To disconnect
---
run `adb disconnect` will disconnect everything including the control hub.
