# Cleanup in case the script dies
cleanup() {
    # kill all processes whose parent is this process
    pkill -P $$
}

for sig in INT QUIT HUP TERM; do
  trap "
    cleanup
    trap - $sig EXIT
    kill -s $sig "'"$$"' "$sig"
done
trap cleanup EXIT

# Print timeout if set, otherwise use default
echo "Timeout set to: $1"

# Run USB & PoE stability stress test
DEPTHAI_PROTOCOL=usb timeout $(($1+30)) ./tests/stability_stress_test $1 &
jobUsb=$!
DEPTHAI_PROTOCOL=tcpip timeout $(($1+30)) ./tests/stability_stress_test $1 &
jobTcpip=$!

# Wait for tests and save result code
wait $jobUsb ; resultUsb=$?
wait $jobTcpip ; resultTcpip=$?

# Print results
echo "Stability test USB: $resultUsb"
echo "Stability test PoE: $resultTcpip"

# If both tests concluded successfully, exit with code 0
if [[ "$resultUsb" == "0"  ]] && [[ "$resultTcpip" == "0" ]]; then
    echo "Success!"
    exit 0
fi
echo "Failed!"
exit 1
