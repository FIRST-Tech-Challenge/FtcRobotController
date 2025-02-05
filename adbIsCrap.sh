#!/bin/zsh
cd ~/Library/Android/sdk/platform-tools
./adb kill-server
./adb disconnect 192.168.43.1
./adb connect 192.168.43.1
echo "press anyting to disconect"
read -n 1
./adb disconnect 192.168.43.1
./adb kill-server

