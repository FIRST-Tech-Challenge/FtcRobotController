package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
@Config
@TeleOp
public class CORobotCodeLM2_V0 extends LinearOpMode {

    public static double MAX_ARM_POWER = 0.5;
    public static double MAX_SLIDE_POWER = 0.7;
    public static int SLIDE_DEPOSIT_POSITION = 4250;
    public static int SLIDE_SPEC_BAR_POSITION = 2250;
    public static int SLIDE_SPEC_CLIP_POSITION = 1750;
    public static int SLIDE_SPEC_GRAB_POSITION = 0;
    public static int ARM_GRAB_POSITION = 505;
    public static int ARM_HOLD_POSITION = 400;
    public static int ARM_TRANSFER_POSITION = 200;
    public static int ARM_SUB_HOLD = 400;
    public static double WRIST_TRANSFER_POSITION = 0.1;
    public static double WRIST_GRAB_POSITION = 0.55;
    public static double ARM_CLAW_FULL_OPEN = 0;
    public static double ARM_CLAW_FULL_CLOSE = 0.65;
    public static double ARM_CLAW_TRANSFER_OPEN = 0.4;
    public static double SPEC_CLAW_OPEN = 0.9;
    public static double SPEC_CLAW_CLOSE = 0.5;
    public static double BUCKET_DEPOSIT_POSITION = 0.27;
    public static double BUCKET_TRANSFER_POSITION = 0.9;
    private PIDFMotorController armController;
    private PIDFMotorController slideController;

    // Define hardware components
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private Servo rightWristServo, specServo, bucketServo;
    private Servo clawIntake;
    private IMU imu;

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

        while (opModeIsActive()) {
            handleDriving();
            slideControl();
            intakeControl();
            bucketControl();
            specClawControl();

            runPIDIterations();
            telemetry.update();
        }
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
        clawIntake = hardwareMap.get(Servo.class, "clawIntake");

        double armTicksInDegrees = 1425.1 / 360.0;
        double slideTicksInDegrees = 537.7 / 360.0;

        // Initialize PIDF controllers for the arm and slide
        armController = new PIDFMotorController(intakeArmMotor, 0.01, 0.23, 0.001, 0.4, armTicksInDegrees, MAX_ARM_POWER);
        slideController = new PIDFMotorController(rightSlideMotor, 0.01, 0.25, 0.001, 0, slideTicksInDegrees, MAX_SLIDE_POWER);

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

    private void slideControl(){
        double inputPosition = gamepad2.left_trigger + gamepad2.right_trigger;
        int slidePosition = (int) (inputPosition / 2 * SLIDE_DEPOSIT_POSITION);
        if (slidePosition != 0) {
            slideController.setTargetPosition(slidePosition);
        } else {
            if (gamepad2.dpad_up) {
                slideController.setTargetPosition(SLIDE_DEPOSIT_POSITION);
            } else if (gamepad2.dpad_down) {
                slideController.setTargetPosition(SLIDE_SPEC_GRAB_POSITION);
            } else if (gamepad2.dpad_left) {
                slideController.setTargetPosition(SLIDE_SPEC_BAR_POSITION);
            } else if (gamepad2.dpad_right) {
                slideController.setTargetPosition(SLIDE_SPEC_CLIP_POSITION);
            }
        }
    }

    private void intakeControl(){
        if (gamepad1.a){
            armController.setTargetPosition(ARM_GRAB_POSITION);
            rightWristServo.setPosition(WRIST_GRAB_POSITION);
            clawIntake.setPosition(ARM_CLAW_FULL_OPEN);
        } else if (gamepad1.b){
            armController.setTargetPosition(ARM_HOLD_POSITION);
            rightWristServo.setPosition(WRIST_TRANSFER_POSITION);
        } else if (gamepad1.y){
            armController.setTargetPosition(ARM_TRANSFER_POSITION);
            rightWristServo.setPosition(WRIST_TRANSFER_POSITION);
        } else if (gamepad1.x){
            armController.setTargetPosition(ARM_SUB_HOLD);
            rightWristServo.setPosition(WRIST_GRAB_POSITION);
        } else if (gamepad1.right_bumper){
            clawIntake.setPosition(ARM_CLAW_FULL_CLOSE);
        } else if (gamepad1.left_bumper){
            clawIntake.setPosition(ARM_CLAW_TRANSFER_OPEN);
        }
    }

    /**
     * Controls the bucket position based on gamepad2 inputs.
     */
    private void bucketControl() {
        if (gamepad2.a) {
            bucketServo.setPosition(BUCKET_TRANSFER_POSITION); // Default position
        } else if (gamepad2.y) {
            bucketServo.setPosition(BUCKET_DEPOSIT_POSITION); // Release position
        }
    }
    private void specClawControl(){
        if (gamepad2.b) {
            specServo.setPosition(SPEC_CLAW_CLOSE); //close spec claw
        }
        if (gamepad2.x) {
            specServo.setPosition(SPEC_CLAW_OPEN); //open spec claw
        }
    }

    private void runPIDIterations() {
        armController.runIteration();
        slideController.runIteration();
    }
}
