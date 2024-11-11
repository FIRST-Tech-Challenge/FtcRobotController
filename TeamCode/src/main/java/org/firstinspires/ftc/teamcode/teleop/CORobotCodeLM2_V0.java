package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * TeleOp mode for controlling the robot. Integrates driving, arm, slide, and intake systems.
 * Uses PIDF controllers to manage motor movements for precise positioning.
 */
@TeleOp
public class CORobotCodeLM2_V0 extends LinearOpMode {

    private PIDFMotorController armController;
    private PIDFMotorController slideController;

    // Define hardware components
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private Servo rightWristServo, specServo, bucketServo;
    private CRServo activeIntake;
    private IMU imu;

    // PIDF values and position constants
    private static final double ARM_TICKS_IN_DEGREES = 1425.1 / 360.0;
    private static final double SLIDE_TICKS_IN_DEGREES = 537.7 / 360.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        initializeHardware();

        // Reset IMU orientation
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu.initialize(imuParameters);

        // Wait for start
        telemetry.addLine("Initialized. Ready to start.");
        telemetry.update();
        waitForStart();



        if (isStopRequested()) return;

        setupPosition();
        while (opModeIsActive()) {
            handleDriving();
            handleArmAndSlideControl();
            handleIntake();
            handleBucketControl();
            handleSpec();

            telemetry.update();
        }
    }

    /**
     * Pre-loop setup sequence to reset arm and slide positions
     */
    private void setupPosition() {
        // Slide motor moves to initial position
        slideController.driveToPosition(700);
        sleep(3000);
        slideController.driveToPosition(0);

        // Position the wrist and intake arm for initial setup
        rightWristServo.setPosition(0.1);
        sleep(1000);
        armController.driveToPosition(-420); // Move arm to initial position
        sleep(1500);

        // Reset encoder and prepare arm for teleop
        armController.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armController.motor.setTargetPosition(0);
        armController.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armController.driveToPosition(300);
        sleep(1000);
    }

    /**
     * Initializes the hardware components and PIDF controllers.
     */
    private void initializeHardware() {
        // Initialize motors and servos
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        DcMotorEx intakeArmMotor = hardwareMap.get(DcMotorEx.class, "intakeArmMotor");
        DcMotorEx rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");

        rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
        specServo = hardwareMap.get(Servo.class, "specServo");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        activeIntake = hardwareMap.get(CRServo.class, "activeIntake");

        // Initialize PIDF controllers for the arm and slide
        armController = new PIDFMotorController(intakeArmMotor, 0, 0, 0, 0, ARM_TICKS_IN_DEGREES);
        slideController = new PIDFMotorController(rightSlideMotor, 0, 0, 0, 0, SLIDE_TICKS_IN_DEGREES);

        // Set directions for drivetrain motors
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
    }

    /**
     * Manages the drivetrain controls using mecanum drive with field-oriented control.
     */
    private void handleDriving() {
        double y = -gamepad1.left_stick_y; // Forward/backward
        double x = gamepad1.left_stick_x;  // Left/right
        double rx = gamepad1.right_stick_x; // Rotation

        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Field-oriented adjustments
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    /**
     * Controls the arm and slide using PIDF control.
     */
    private void handleArmAndSlideControl() {
        // Move arm to position based on button presses
        if (gamepad1.y) {
            armController.driveToPosition(300);  // Set target position for arm
            sleep(500);
            rightWristServo.setPosition(0.55);   // Example position
            bucketServo.setPosition(0.27);
        } else if (gamepad1.a) {
            armController.driveToPosition(80);
            rightWristServo.setPosition(0.5);
            sleep(200);
            rightWristServo.setPosition(0.43);
        } else if (gamepad1.x) {
            armController.driveToPosition(200);
            rightWristServo.setPosition(0.5);
        }

        // Manual control for slide using gamepad2's left stick
        double slidePower = gamepad2.left_stick_y;
        slideController.driveToPosition((int) (slidePower * 1000));  // Adjust target as needed
    }

    /**
     * Controls the intake system based on gamepad inputs.
     */
    private void handleIntake() {
        if (gamepad1.right_bumper) {
            activeIntake.setPower(-1);
        } else if (gamepad1.b) {
            activeIntake.setPower(1);
        } else {
            activeIntake.setPower(0);
        }
    }

    /**
     * Controls the bucket position based on gamepad2 inputs.
     */
    private void handleBucketControl() {
        if (gamepad2.a) {
            bucketServo.setPosition(0.27); // Default position
        } else if (gamepad2.y) {
            bucketServo.setPosition(0.9); // Release position
        }
    }
    private void handleSpec(){
        if (gamepad2.b) {
            specServo.setPosition(0.2); //open spec claw
        }
        if (gamepad2.x) {
            specServo.setPosition(0.8); //closed spec claw
        }
    }
}
