pushd "%userprofile%\AppData\Local\Android\sdk\platform-tools"

adb disconnect

adb tcpip 5555
adb connect 192.168.43.1

popd

PAUSE