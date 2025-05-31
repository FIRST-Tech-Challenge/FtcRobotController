Setup ADB tool
---
1. Find out the SDK_HOME. In android studio, in hamburger menu, goto tools/sdk manager, write down android sdk location shows in the dialog.
2. Settings > System > Advanced System Settings > Environment Variables, in current user or system section, select path, then click edit, add new item with value $SDK_HOME/platform-tools, replace the $SDK_HOME with value saved from above step
3. save the change, and restart the terminal or android studio
4. Test by typing adb in a terminal

Why
---
To deploy and capture latest logcat output when robot is moving
To push code to device without direct connection

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

Dashboard
---
```
http://192.168.43.1:8080/dash
```