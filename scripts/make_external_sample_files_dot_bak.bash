parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )

cd "$parent_path"/../FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples

for file in ./*.java; do
    mv "$file" "${file%.java}.bak"
done
