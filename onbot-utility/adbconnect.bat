@cd platform-tools

netsh wlan connect name=7078-RC interface="Wi-Fi 2"

adb devices

adb usb
timeout 4
adb devices

adb tcpip 5555
timeout 4
adb devices
adb connect 192.168.43.1:5555
adb devices
@REM adb connect 192.168.49.1
@cd ..

