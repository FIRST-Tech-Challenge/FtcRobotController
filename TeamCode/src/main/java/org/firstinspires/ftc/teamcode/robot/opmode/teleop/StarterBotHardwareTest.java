package org.firstinspires.ftc.teamcode.robot.opmode.teleop;

import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * Tests every motor and servo used by BioBuzzStarterbotTeleopMecanum, one at a time,
 * and shows which hub and port each one is plugged into.
 *
 * Every device gets the same direction and setup as in the real teleop, so positive
 * power here should move each part the way the teleop expects: drive wheels forward,
 * intake and intake servos pulling in, windmill feeding the launcher, launcher shooting.
 * If one goes the wrong way, fix its direction in the teleop (or in Constants for the
 * drive motors) and here.
 *
 * Controls (gamepad 1):
 *   dpad up / down   pick the device to test (only the picked one ever runs)
 *   left stick Y     run it, up = positive power
 *   A / B            run it at +TEST_POWER / -TEST_POWER
 *   Y                launcher only: spin up to the teleop's target velocity
 *   X                motors only: reset the encoder
 *
 * Any device missing from the robot configuration is listed as MISSING and skipped;
 * the rest can still be tested.
 */
@TeleOp(name = "StarterBot Hardware Test", group = "Test")
@Disabled
public class StarterBotHardwareTest extends LinearOpMode {

    private static final double TEST_POWER = 0.5;
    private static final double STICK_DEADBAND = 0.05;

    // Keep these in sync with BioBuzzStarterbotTeleopMecanum
    private static final int LAUNCHER_TARGET_VELOCITY = 1250;
    private static final int LAUNCHER_MIN_VELOCITY = 1200;
    private static final String LAUNCHER_NAME = "launcher";

    /** One motor or continuous-rotation servo from the robot configuration. */
    private static class Device {
        final String label;
        final String configName;
        DcMotorEx motor;
        CRServo servo;
        String error;

        Device(String label, String configName) {
            this.label = label;
            this.configName = configName;
        }

        boolean found() {
            return motor != null || servo != null;
        }

        void setPower(double power) {
            if (motor != null) motor.setPower(power);
            if (servo != null) servo.setPower(power);
        }

        /** Hub and port, e.g. "Control Hub port 2". */
        String location() {
            if (motor != null) {
                return motor.getController().getDeviceName() + " motor port " + motor.getPortNumber();
            }
            return servo.getController().getDeviceName() + " servo port " + servo.getPortNumber();
        }
    }

    @Override
    public void runOpMode() {
        MecanumConfig drive = Constants.drivetrainConfig;
        List<Device> devices = new ArrayList<>();

        devices.add(motor("Front left drive", drive.frontLeftName.get(), drive.frontLeftDirection.get()));
        devices.add(motor("Front right drive", drive.frontRightName.get(), drive.frontRightDirection.get()));
        devices.add(motor("Back left drive", drive.backLeftName.get(), drive.backLeftDirection.get()));
        devices.add(motor("Back right drive", drive.backRightName.get(), drive.backRightDirection.get()));
        devices.add(motor("Intake", "intake", DcMotorSimple.Direction.FORWARD));

        Device launcher = motor("Launcher", LAUNCHER_NAME, DcMotorSimple.Direction.FORWARD);
        if (launcher.found()) {
            launcher.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher.motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(40, 0, 0, 12.5));
        }
        devices.add(launcher);

        devices.add(servo("Left intake servo", "left_intake_servo", DcMotorSimple.Direction.FORWARD));
        devices.add(servo("Right intake servo", "right_intake_servo", DcMotorSimple.Direction.REVERSE));
        devices.add(servo("Windmill servo", "windmillServo", DcMotorSimple.Direction.REVERSE));

        int missing = 0;
        for (Device d : devices) if (!d.found()) missing++;
        telemetry.addLine("Ready. Dpad picks a device, left stick or A/B runs it.");
        if (missing > 0) telemetry.addData("Missing from configuration", missing);
        addDeviceList(devices, -1);
        telemetry.update();
        waitForStart();

        int selected = 0;
        boolean lastUp = false;
        boolean lastDown = false;
        boolean lastX = false;

        while (opModeIsActive()) {
            if (gamepad1.dpad_down && !lastDown) selected = (selected + 1) % devices.size();
            if (gamepad1.dpad_up && !lastUp) selected = (selected + devices.size() - 1) % devices.size();
            boolean xPressed = gamepad1.x && !lastX;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastX = gamepad1.x;

            Device current = devices.get(selected);

            double power = -gamepad1.left_stick_y;
            if (Math.abs(power) < STICK_DEADBAND) power = 0;
            if (gamepad1.a) power = TEST_POWER;
            if (gamepad1.b) power = -TEST_POWER;
            boolean velocityTest = current == launcher && current.found() && gamepad1.y;

            // Only the selected device runs; everything else is held at zero
            for (Device d : devices) {
                if (!d.found() || (d == current && velocityTest)) continue;
                d.setPower(d == current ? power : 0);
            }
            if (velocityTest) launcher.motor.setVelocity(LAUNCHER_TARGET_VELOCITY);

            if (xPressed && current.motor != null) {
                DcMotor.RunMode mode = current.motor.getMode();
                current.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                current.motor.setMode(mode);
            }

            telemetry.addData("Testing", "%s (\"%s\")", current.label, current.configName);
            if (!current.found()) {
                telemetry.addData("Status", "MISSING: %s", current.error);
            } else {
                telemetry.addData("Plugged into", current.location());
                if (current.motor != null) {
                    DcMotorEx m = current.motor;
                    telemetry.addData("Power", "%.2f", m.getPower());
                    telemetry.addData("Encoder", "%d  (X to reset)", m.getCurrentPosition());
                    telemetry.addData("Velocity (ticks/s)", "%.0f", m.getVelocity());
                    telemetry.addData("Current (A)", "%.2f", m.getCurrent(CurrentUnit.AMPS));
                    if (current == launcher) {
                        telemetry.addData("Launcher test", velocityTest
                                ? (m.getVelocity() > LAUNCHER_MIN_VELOCITY ? "AT SPEED" : "spinning up...")
                                : "hold Y to spin up to " + LAUNCHER_TARGET_VELOCITY);
                    }
                } else {
                    telemetry.addData("Power", "%.2f", current.servo.getPower());
                }
            }
            addDeviceList(devices, selected);
            telemetry.update();
        }

        for (Device d : devices) if (d.found()) d.setPower(0);
    }

    private Device motor(String label, String name, DcMotorSimple.Direction direction) {
        Device d = new Device(label, name);
        try {
            d.motor = hardwareMap.get(DcMotorEx.class, name);
            d.motor.setDirection(direction);
            d.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            d.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            d.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            d.motor = null;
            d.error = e.getMessage();
        }
        return d;
    }

    private Device servo(String label, String name, DcMotorSimple.Direction direction) {
        Device d = new Device(label, name);
        try {
            d.servo = hardwareMap.get(CRServo.class, name);
            d.servo.setDirection(direction);
            d.servo.setPower(0);
        } catch (Exception e) {
            d.servo = null;
            d.error = e.getMessage();
        }
        return d;
    }

    /** Lists every device and where it's plugged in, marking the selected one. */
    private void addDeviceList(List<Device> devices, int selected) {
        telemetry.addLine();
        telemetry.addLine("All devices:");
        for (int i = 0; i < devices.size(); i++) {
            Device d = devices.get(i);
            telemetry.addLine(String.format("%s %s (\"%s\"): %s",
                    i == selected ? ">>" : "   ", d.label, d.configName,
                    d.found() ? d.location() : "MISSING"));
        }
    }
}
