package org.firstinspires.ftc.team417_CENTERSTAGE.tuning;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import java.util.ArrayList;
import java.util.List;

    /**
     * Wrapper for tracking button press state.
     */
    class Buttons {
        final private Gamepad gamepad;
        private boolean a;
        private boolean b;
        private boolean dpad_up;
        private boolean dpad_down;
        Buttons(Gamepad gamepad) {
            this.gamepad = gamepad;
        }
        boolean select() {
            boolean result = a && !gamepad.a;
            a = gamepad.a;
            return result;
        }
        boolean cancel() {
            boolean result = b && !gamepad.b;
            b = gamepad.b;
            return result;
        }
        boolean up() {
            boolean result = dpad_up && !gamepad.dpad_up;
            dpad_up = gamepad.dpad_up;
            return result;
        }
        boolean down() {
            boolean result = dpad_down && !gamepad.dpad_down;
            dpad_down = gamepad.dpad_down;
            return result;
        }
    }
    @TeleOp
    public class ConfigurationTester extends LinearOpMode {
        ArrayList<String> deviceNames = new ArrayList<>();
        Buttons buttons;
        /**
         *
         */
        int menu(String header, List<String> options, int current, boolean topmost) {
            while (isActive()) {
                if (header != null) {
                    telemetry.addLine(header);
                }
                for (int i = 0; i < options.size(); i++) {
                    String cursor = (i == current) ? ">" : " ";
                    telemetry.addLine(cursor + options.get(i));
                }
                telemetry.update();
                if (buttons.up()) {
                    current--;
                    if (current < 0)
                        current = options.size() - 1;
                }
                if (buttons.down()) {
                    current++;
                    if (current == options.size())
                        current = 0;
                }
                if (buttons.cancel() && !topmost)
                    return -1;
                if (buttons.select())
                    return current;
            }
            return topmost ? 0 : -1;
        }
        boolean isActive() {
            return opModeIsActive();
        }
        boolean notCancelled() {
            telemetry.update();
            return isActive() && !buttons.cancel();
        }
        void commonHeader(String deviceName, HardwareDevice device) {
            telemetry.addLine(String.format("Name: %s", deviceName));
            telemetry.addLine(device.getDeviceName());
            telemetry.addLine(device.getConnectionInfo());
            telemetry.addLine(String.format("Version: %d", device.getVersion()));
            telemetry.addLine(String.format("Manufacturer: %s", device.getManufacturer().name()));
        }
        void testMotor(String deviceName) {
            DcMotor motor = (DcMotor) hardwareMap.get(deviceName);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            do {
                commonHeader(deviceName, motor);
                telemetry.addLine(String.format("Encoder position: %d", motor.getCurrentPosition()));
                telemetry.addLine("Right trigger to spin forward, left to spin backward.");
                double power = gamepad1.right_trigger - gamepad1.left_trigger;
                telemetry.addLine(String.format("Power: %.2f", power));
                motor.setPower(power);
            } while (notCancelled());
        }
        void testDistance(String deviceName) {
            DistanceSensor distance = (DistanceSensor) hardwareMap.get(deviceName);
            do {
                commonHeader(deviceName, distance);
                telemetry.addLine(String.format("Distance CM: %.2f", distance.getDistance(DistanceUnit.CM)));
            } while (notCancelled());
        }
        void testGeneric(String deviceName) {
            HardwareDevice device = hardwareMap.get(deviceName);
            do {
                commonHeader(deviceName, device);
            } while (notCancelled());
        }
        void testIMU(String deviceName) {
            IMU imu = (IMU) hardwareMap.get(deviceName);
            do {
                commonHeader(deviceName, imu);
                YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
                telemetry.addLine(String.format("Yaw: %.2f, Pitch: %.2f, Roll: %.2f (degrees)",
                        angles.getYaw(AngleUnit.DEGREES),
                        angles.getPitch(AngleUnit.DEGREES),
                        angles.getRoll(AngleUnit.DEGREES)));
            } while (notCancelled());
        }
        // @@@ Deprecated:
        void addDeviceNames(Class<?> classType) {
            for (String name: hardwareMap.getAllNames(DcMotor.class)) {
                if (!deviceNames.contains(name))
                    deviceNames.add(name);
            }
        }
        @Override
        public void runOpMode() {
            telemetry.addLine("Press START to test the current configuration");
            telemetry.addLine("");
            telemetry.addLine("A to select, B to cancel, dpad to navigate.");
            telemetry.update();
            for (String name: hardwareMap.getAllNames(DcMotor.class)) {
                if (!deviceNames.contains(name))
                    deviceNames.add(name);
            }
            for (String name: hardwareMap.getAllNames(IMU.class)) {
                if (!deviceNames.contains(name))
                    deviceNames.add(name);
            }
            for (String name: hardwareMap.getAllNames(DistanceSensor.class)) {
                if (!deviceNames.contains(name))
                    deviceNames.add(name);
            }
            for (HardwareDevice device: hardwareMap) {
                for (String name: hardwareMap.getAllNames(device.getClass())) {
                    if (!deviceNames.contains(name))
                        deviceNames.add(name);
                }
            }
            waitForStart();
            // We can now initialized the gamepad tracker:
            buttons = new Buttons(gamepad1);
            ArrayList<String> options = new ArrayList<>();
            for (String name: deviceNames) {
                HardwareDevice device = hardwareMap.get(name);
                options.add(String.format("%s: %s", device.getClass().getSimpleName(), name));
            }
            int selection = 0;
            while (isActive()) {
                selection = menu("", options, selection, true);
                String deviceName = deviceNames.get(selection);
                HardwareDevice device = hardwareMap.get(deviceName);
                if (device instanceof DcMotor) {
                    testMotor(deviceName);
                } else if (device instanceof DistanceSensor) {
                    testDistance(deviceName);
                } else if (device instanceof IMU) {
                    testIMU(deviceName);
                } else {
                    testGeneric(deviceName);
                }
            }
        }
    }

