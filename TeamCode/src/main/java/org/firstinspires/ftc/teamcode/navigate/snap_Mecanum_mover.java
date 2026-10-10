package org.firstinspires.ftc.teamcode.navigate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class snap_Mecanum_mover {
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

    private final DcMotor frontLeftDrive;
    private final DcMotor frontRightDrive;
    private final DcMotor backLeftDrive;
    private final DcMotor backRightDrive;
    private final IMU imu;

    private boolean snapTurnActive = false;
    private double targetHeadingRadians = 0.0;
    private boolean leftTriggerWasPressed = false;
    private boolean rightTriggerWasPressed = false;
    private double previousHeadingErrorRadians = 0.0;
    private long previousSnapUpdateTimeNanos = 0L;
    private double lastTurnRateRadiansPerSecond = 0.0;
    private boolean snapTurnSettling = false;
    private int settledLoopCount = 0;

    public snap_Mecanum_mover(
            DcMotor frontLeftDrive,
            DcMotor frontRightDrive,
            DcMotor backLeftDrive,
            DcMotor backRightDrive,
            IMU imu
    ) {
        this.frontLeftDrive = frontLeftDrive;
        this.frontRightDrive = frontRightDrive;
        this.backLeftDrive = backLeftDrive;
        this.backRightDrive = backRightDrive;
        this.imu = imu;
    }

    public void loop(Gamepad gamepad1, Telemetry telemetry, boolean snap_to_tag) {
        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");
        telemetry.addLine("Left trigger snaps to 90° clockwise from field north");
        telemetry.addLine("Right trigger snaps to 90° counterclockwise from field north");
        //if apriltag is foudn and diver is snapping to it, then return right away
        if (snap_to_tag) {
            return;
        }
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
            rotateCommand = -getSnapTurnCommand(telemetry);
        }

        telemetry.addData("Heading (deg)", Math.toDegrees(getCurrentHeadingRadians()));
        telemetry.addData("Snap turn", snapTurnActive ? "ACTIVE" : "OFF");
        if (snapTurnActive) {
            telemetry.addData("Target heading (deg)", Math.toDegrees(targetHeadingRadians));
            telemetry.addData("Heading error (deg)", Math.toDegrees(AngleUnit.normalizeRadians(targetHeadingRadians - getCurrentHeadingRadians())));
            telemetry.addData("Turn rate (deg/s)", Math.toDegrees(lastTurnRateRadiansPerSecond));
            telemetry.addData("Settling", snapTurnSettling ? "YES" : "NO");
        }

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

    private double getSnapTurnCommand(Telemetry telemetry) {
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

    public void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(theta - getCurrentHeadingRadians());

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }

    public void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}

