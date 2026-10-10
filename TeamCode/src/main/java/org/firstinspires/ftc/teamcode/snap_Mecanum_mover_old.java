package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@TeleOp(name = "Robot: snap Mecanum Drive", group = "Robot")

public class snap_Mecanum_mover_old extends OpMode {
    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    private static final double RIGHT_TRIGGER_TARGET_RADIANS = -Math.PI / 2.0;
    private static final double LEFT_TRIGGER_TARGET_RADIANS = Math.PI / 2.0;
    private static final double TURN_STOP_DEADBAND_RADIANS = Math.toRadians(1.0);
    private static final double SETTLE_TOLERANCE_RADIANS = Math.toRadians(2.0);
    private static final double BRAKE_ZONE_RADIANS = Math.toRadians(45.0);
    private static final double FINE_CONTROL_ZONE_RADIANS = Math.toRadians(6.0);
    private static final double SNAP_TURN_KP = 0.70;
    //private static final double SNAP_TURN_KD = 0.60;
    private static final double SNAP_TURN_KD = 0.35;
    private static final double SNAP_TURN_MAX_POWER = 0.70;
    private static final double SNAP_TURN_BRAKE_MAX_POWER = 0.70;
    private static final double SNAP_TURN_SETTLE_MAX_POWER = 0.14;
    private static final double SNAP_TURN_MIN_POWER = 0.14;
    private static final double SNAP_TURN_BRAKE_MIN_POWER = 0.08;
    private static final double MAX_SETTLED_TURN_RATE_RADIANS = Math.toRadians(5.0);
    private static final int REQUIRED_SETTLED_LOOPS = 7;
    private static final double TRIGGER_THRESHOLD = 0.5;
    private static final double MANUAL_ROTATE_DEADBAND = 0.05;

    private boolean snapTurnActive = false;
    private double targetHeadingRadians = 0.0;
    private boolean leftTriggerWasPressed = false;
    private boolean rightTriggerWasPressed = false;
    private double previousHeadingErrorRadians = 0.0;
    private long previousSnapUpdateTimeNanos = 0L;
    private double lastTurnRateRadiansPerSecond = 0.0;
    private boolean snapTurnSettling = false;
    private int settledLoopCount = 0;

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void loop() {
        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");
        telemetry.addLine("Left trigger snaps to 90° clockwise from field north");
        telemetry.addLine("Right trigger snaps to 90° counterclockwise from field north");

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing

        if (gamepad1.a) {
            imu.resetYaw();
            snapTurnActive = false;
        }

        boolean leftTriggerPressed = gamepad1.left_trigger > TRIGGER_THRESHOLD;
        boolean rightTriggerPressed = gamepad1.right_trigger > TRIGGER_THRESHOLD;

        if (leftTriggerPressed && !rightTriggerWasPressed) {
            startSnapTurn(LEFT_TRIGGER_TARGET_RADIANS);
        } else if (rightTriggerPressed && !leftTriggerWasPressed) {
            startSnapTurn(RIGHT_TRIGGER_TARGET_RADIANS);
        }

        leftTriggerWasPressed = leftTriggerPressed;
        rightTriggerWasPressed = rightTriggerPressed;

        double rotateCommand = gamepad1.right_stick_x;
        if (Math.abs(rotateCommand) > MANUAL_ROTATE_DEADBAND) {
            snapTurnActive = false;
        } else if (snapTurnActive) {
            rotateCommand = -getSnapTurnCommand();
        }

        telemetry.addData("Heading (deg)", Math.toDegrees(getCurrentHeadingRadians()));
        telemetry.addData("Snap turn", snapTurnActive ? "ACTIVE" : "OFF");
        if (snapTurnActive) {
            telemetry.addData("Target heading (deg)", Math.toDegrees(targetHeadingRadians));
            telemetry.addData("Heading error (deg)", Math.toDegrees(AngleUnit.normalizeRadians(targetHeadingRadians - getCurrentHeadingRadians())));
            telemetry.addData("Turn rate (deg/s)", Math.toDegrees(lastTurnRateRadiansPerSecond));
            telemetry.addData("Settling", snapTurnSettling ? "YES" : "NO");
        }

        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
        if (gamepad1.left_bumper) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotateCommand);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotateCommand);
        }
    }

    private void startSnapTurn(double headingRadians) {
        targetHeadingRadians = AngleUnit.normalizeRadians(headingRadians);
        previousHeadingErrorRadians = AngleUnit.normalizeRadians(targetHeadingRadians - getCurrentHeadingRadians());
        previousSnapUpdateTimeNanos = System.nanoTime();
        lastTurnRateRadiansPerSecond = 0.0;
        snapTurnSettling = false;
        settledLoopCount = 0;
        snapTurnActive = true;
    }

    private double getSnapTurnCommand() {
        double headingError = AngleUnit.normalizeRadians(targetHeadingRadians - getCurrentHeadingRadians());
        telemetry.addData("heading error value", Math.abs(headingError));
        telemetry.addData("deadband", TURN_STOP_DEADBAND_RADIANS);
        telemetry.addData("settled loop count", settledLoopCount);




        long nowNanos = System.nanoTime();
        double deltaTimeSeconds = (nowNanos - previousSnapUpdateTimeNanos) / 1_000_000_000.0;

        if (deltaTimeSeconds <= 0.0) {
            deltaTimeSeconds = 0.001;
        }

        double errorDeltaRadians = AngleUnit.normalizeRadians(headingError - previousHeadingErrorRadians);
        double errorRate = errorDeltaRadians / deltaTimeSeconds;
        lastTurnRateRadiansPerSecond = -errorRate;

        if (Math.abs(headingError) <= TURN_STOP_DEADBAND_RADIANS) {
            snapTurnActive = false;
            snapTurnSettling = false;
            settledLoopCount = 0;
            previousHeadingErrorRadians = headingError;
            previousSnapUpdateTimeNanos = nowNanos;
            lastTurnRateRadiansPerSecond = 0.0;
            return 0.0;
        }

        boolean overshotTarget = Math.signum(previousHeadingErrorRadians) != Math.signum(headingError)
                && Math.abs(previousHeadingErrorRadians) > TURN_STOP_DEADBAND_RADIANS;
        if (overshotTarget || Math.abs(headingError) <= FINE_CONTROL_ZONE_RADIANS) {
            snapTurnSettling = true;
        }

        boolean headingWithinTolerance = Math.abs(headingError) <= SETTLE_TOLERANCE_RADIANS;
        boolean turnRateIsLow = Math.abs(lastTurnRateRadiansPerSecond) <= MAX_SETTLED_TURN_RATE_RADIANS;

        if (headingWithinTolerance && turnRateIsLow) {
            settledLoopCount++;
        } else {
            settledLoopCount = 0;
        }

        if (settledLoopCount >= REQUIRED_SETTLED_LOOPS
                && Math.abs(headingError) <= TURN_STOP_DEADBAND_RADIANS) {
            snapTurnActive = false;
            snapTurnSettling = false;
            settledLoopCount = 0;
            previousHeadingErrorRadians = headingError;
            previousSnapUpdateTimeNanos = nowNanos;
            lastTurnRateRadiansPerSecond = 0.0;
            return 0.0;
        }

        double rotateCommand = (headingError * SNAP_TURN_KP) + (errorRate * SNAP_TURN_KD);

        double absoluteError = Math.abs(headingError);
        double maxAllowedPower;
        double minAllowedPower = 0.0;

        if (snapTurnSettling) {
            maxAllowedPower = SNAP_TURN_SETTLE_MAX_POWER;
        } else if (absoluteError <= BRAKE_ZONE_RADIANS) {
            double brakeBlend = Range.clip(absoluteError / BRAKE_ZONE_RADIANS, 0.0, 1.0);
            maxAllowedPower = SNAP_TURN_BRAKE_MAX_POWER
                    + (SNAP_TURN_MAX_POWER - SNAP_TURN_BRAKE_MAX_POWER) * brakeBlend;
            minAllowedPower = SNAP_TURN_BRAKE_MIN_POWER * brakeBlend;
        } else {
            maxAllowedPower = SNAP_TURN_MAX_POWER;
            minAllowedPower = SNAP_TURN_MIN_POWER;
        }

        rotateCommand = Range.clip(rotateCommand, -maxAllowedPower, maxAllowedPower);

        if (minAllowedPower > 0.0 && Math.abs(rotateCommand) < minAllowedPower) {
            rotateCommand = Math.copySign(minAllowedPower, rotateCommand);
        }

        previousHeadingErrorRadians = headingError;
        previousSnapUpdateTimeNanos = nowNanos;

        return rotateCommand;
    }

    private double getCurrentHeadingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                getCurrentHeadingRadians());

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}

