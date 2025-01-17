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

@TeleOp(name = "Syborgs")
@SuppressWarnings({"unused", "FieldCanBeLocal"})
public class Syborgs extends LinearOpMode {

    // Hardware declarations
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotor armMotor, liftMotor, hangMotor;
    private CRServo intake;
    private Servo wrist;

    // Arm constants
    private static final double ARM_TICKS_PER_DEGREE = 28.0 * 250047.0 / 4913.0 * 100.0 / 20.0 / 360.0;
    private static final double ARM_COLLAPSED_INTO_ROBOT = 0;
    private static final double ARM_COLLECT = 0 * ARM_TICKS_PER_DEGREE;
    private static final double ARM_CLEAR_BARRIER = 15 * ARM_TICKS_PER_DEGREE;
    private static final double ARM_SCORE_SPECIMEN = 90 * ARM_TICKS_PER_DEGREE;
    private static final double ARM_SCORE_SAMPLE_IN_LOW = 90 * ARM_TICKS_PER_DEGREE;
    private static final double ARM_ATTACH_HANGING_HOOK = 110 * ARM_TICKS_PER_DEGREE;
    private static final double ARM_WINCH_ROBOT = 10 * ARM_TICKS_PER_DEGREE;
    private static final double FUDGE_FACTOR = 15.0 * ARM_TICKS_PER_DEGREE;
    private double armPosition = ARM_COLLAPSED_INTO_ROBOT;
    private double armPositionFudgeFactor;

    // Lift constants
    private static final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;
    private static final double LIFT_COLLAPSED = 0.0 * LIFT_TICKS_PER_MM;
    private static final double LIFT_SCORING_IN_LOW_BASKET = 0.0 * LIFT_TICKS_PER_MM;
    private static final double LIFT_SCORING_IN_HIGH_BASKET = 480.0 * LIFT_TICKS_PER_MM;
    private double liftPosition = LIFT_COLLAPSED;
    private double cycleTime = 0, loopTime = 0, oldTime = 0, armLiftComp = 0;
    private static final double ARM_LIFT_COMPENSATION_FACTOR = 0.25568;

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleDriveControl();
            handleIntakeControl();
            handleArmControl();
            handleLiftControl();
            handleHangControl();
            handleTelemetry();
        }
    }

    private void initializeHardware() {
        // Initialize motors
        frontLeftDrive = hardwareMap.dcMotor.get("frontLeftDrive");
        frontRightDrive = hardwareMap.dcMotor.get("frontRightDrive");
        backLeftDrive = hardwareMap.dcMotor.get("backLeftDrive");
        backRightDrive = hardwareMap.dcMotor.get("backRightDrive");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        hangMotor = hardwareMap.dcMotor.get("hangMotor");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Initialize servos
        intake = hardwareMap.crservo.get("intake");
        wrist = hardwareMap.servo.get("wrist");

        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_OUT);

        // Initialize IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)
        );
        imu.initialize(parameters);
    }

    private void handleDriveControl() {
        // Drive control logic
        // ...
    }

    private void handleIntakeControl() {
        // Intake control logic
        // ...
    }

    private void handleArmControl() {
        // Arm control logic
        // ...
    }

    private void handleLiftControl() {
        // Lift control logic
        // ...
    }

    private void handleHangControl() {
        // Hang motor control logic
        hangMotor.setPower(-gamepad2.left_stick_y);
    }

    private void handleTelemetry() {
        // Telemetry updates
        telemetry.addData("ArmSubsystem Target Position", armMotor.getTargetPosition());
        telemetry.addData("ArmSubsystem Encoder", armMotor.getCurrentPosition());
        telemetry.addData("LiftSubsystem Variable", liftPosition);
        telemetry.addData("LiftSubsystem Target Position", liftMotor.getTargetPosition());
        telemetry.addData("LiftSubsystem Current Position", liftMotor.getCurrentPosition());
        if (liftMotor instanceof DcMotorEx) {
            telemetry.addData("LiftSubsystem motor current", ((DcMotorEx) liftMotor).getCurrent(CurrentUnit.AMPS));
        }
        telemetry.update();
    }
}
