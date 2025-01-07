package org.firstinspires.ftc.teamcode.deprecated;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
KEY IMPROVEMENTS:
1.Initialization of motors + imu setup are more modular
2.Less unnecessary code, especially for arm movement and motors
3.Added better parameters for motor control, better velocity control and power
4.Telemetry data improved

Code reduced from 226 to 212 lines

**If we use this, change class name below to Syborgs**
 */

@TeleOp(name = "Syborgs2")
@SuppressWarnings("unused")
public class Syborgs2 extends LinearOpMode {
    private DcMotor
            frontLeftDrive,
            frontRightDrive,
            backLeftDrive,
            backRightDrive;
    private DcMotor armMotor;
    private DcMotor liftMotor;
    private DcMotor hangMotor;
    private CRServo intake;
    private Servo wrist;

    final double ARM_TICKS_PER_DEGREE = 28.0 * 250047.0 / 4913.0 * 100.0 / 20.0 / 360.0;
    final double ARM_COLLECT = 0 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK = 110 * ARM_TICKS_PER_DEGREE;
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;
    final double WRIST_FOLDED_IN = 1.0 / 6.0;
    final double WRIST_FOLDED_OUT = 0.5;
    final double FUDGE_FACTOR = 15.0 * ARM_TICKS_PER_DEGREE;

    double armPosition = ARM_COLLECT;
    double armPositionFudgeFactor;
    final double LIFT_SCORING_IN_HIGH_BASKET = 480.0;
    double liftPosition = 0;
    double cycleTime = 0, loopTime = 0, oldTime = 0;

    @Override
    public void runOpMode() {
        // Initialize hardware components
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        hangMotor = hardwareMap.dcMotor.get("hangMotor");

        // Motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Motor Zero Power Behavior
        setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize arm and lift motor encoders
        initializeMotors();

        intake = hardwareMap.crservo.get("IN");
        wrist = hardwareMap.servo.get("WR");

        // Set intake and wrist positions
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_OUT);

        // IMU Initialization
        IMU imu = hardwareMap.get(IMU.class, "imu");
        initializeIMU(imu);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update robot's movement
            handleDriveMovement(imu);

            // Control the intake and arm positions
            handleIntakeAndArmControl();

            // Update the arm position and lift motor position
            updateArmAndLiftPositions();

            // Control the hang motor
            hangMotor.setPower(-gamepad2.left_stick_y);

            // Update cycle time and telemetry
            loopTime = getRuntime();
            cycleTime = loopTime - oldTime;
            oldTime = loopTime;

            updateTelemetry();
        }
    }

    private void setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeftDrive.setZeroPowerBehavior(behavior);
        frontRightDrive.setZeroPowerBehavior(behavior);
        backLeftDrive.setZeroPowerBehavior(behavior);
        backRightDrive.setZeroPowerBehavior(behavior);
        armMotor.setZeroPowerBehavior(behavior);
        hangMotor.setZeroPowerBehavior(behavior);
    }

    private void initializeMotors() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void initializeIMU(IMU imu) {
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)
        );
        imu.initialize(parameters);
    }

    private void handleDriveMovement(IMU imu) {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rotation = gamepad1.right_stick_x;

        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX *= 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotation), 1);
        setDrivePower(rotY, rotX, rotation, denominator);
    }

    private void setDrivePower(double rotY, double rotX, double rotation, double denominator) {
        double frontLeftPower = (rotY + rotX + rotation) / denominator;
        double frontRightPower = (rotY - rotX - rotation) / denominator;
        double backLeftPower = (rotY - rotX + rotation) / denominator;
        double backRightPower = (rotY + rotX - rotation) / denominator;

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    private void handleIntakeAndArmControl() {
        if (gamepad1.left_bumper) {
            intake.setPower(INTAKE_COLLECT);
        } else if (gamepad1.right_bumper) {
            intake.setPower(INTAKE_OFF);
        } else if (gamepad1.y) {
            intake.setPower(INTAKE_DEPOSIT);
        }

        armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger - gamepad2.left_trigger);

        if (gamepad1.a) armPosition = ARM_COLLECT;
        else if (gamepad1.b) armPosition = ARM_CLEAR_BARRIER;
        else if (gamepad1.x) armPosition = ARM_SCORE_SAMPLE_IN_LOW;
        else if (gamepad1.dpad_left) armPosition = ARM_COLLECT;
        else if (gamepad1.dpad_right) armPosition = ARM_ATTACH_HANGING_HOOK;
        else if (gamepad1.dpad_up) armPosition = ARM_ATTACH_HANGING_HOOK;
        else if (gamepad1.dpad_down) armPosition = ARM_SCORE_SPECIMEN;

        wrist.setPosition(WRIST_FOLDED_IN);
    }

    private void updateArmAndLiftPositions() {
        armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));
        if (armMotor instanceof DcMotorEx) {
            ((DcMotorEx) armMotor).setVelocity(2100);
        }
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setTargetPosition((int) liftPosition);
        if (liftMotor instanceof DcMotorEx) {
            ((DcMotorEx) liftMotor).setVelocity(2100);
        }
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void updateTelemetry() {
        telemetry.addData("ArmSubsystem Target Position", armMotor.getTargetPosition());
        telemetry.addData("ArmSubsystem Encoder", armMotor.getCurrentPosition());
        telemetry.addData("LiftSubsystem Position", liftPosition);
        telemetry.addData("LiftSubsystem Target Position", liftMotor.getTargetPosition());
        telemetry.addData("LiftSubsystem Current Position", liftMotor.getCurrentPosition());
        telemetry.addData("LiftSubsystem Motor Current", ((DcMotorEx) liftMotor).getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}
