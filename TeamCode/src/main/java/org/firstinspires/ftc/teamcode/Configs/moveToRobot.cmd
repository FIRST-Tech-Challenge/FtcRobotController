@echo off
rem this script requires to have the adb platform-tools folder adjacent to it
rem this script also requires that only a single control hub is connected via usb to this device and no other android devices are connected

rem push all config files to control hub
for %%f in (*.xml) do (
    .\platform-tools\adb push "%%f" /sdcard/FIRST/
)