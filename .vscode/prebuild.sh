#!/bin/bash

# Terminate the ADB server (if running)
adb kill-server

# Start the ADB seerver
adb start-server

# Check the operating system and run platform-specific commands
if [[ "$OSTYPE" == "darwin"* ]]; then
  # Mac-specific commands
  adb connect 192.168.43.1 # change the ip based on your RC's wifi ip
  ./gradlew installRelease

elif [[ "$OSTYPE" == "mysys"* ]]; then
  # Windows specific commands
  adb tcpip 5555
  adb connect 192.168.43.1 # change the ip based on your RC's wifi ip
  .\gradlew installRelease
  
else
  echo "Unsupported operating system"
fi