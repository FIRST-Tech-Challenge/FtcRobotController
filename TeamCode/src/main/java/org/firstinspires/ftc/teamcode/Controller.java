package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "controller movement", group = "SA_FTC")
public class Controller extends LinearOpMode {
    // Movement
    final double AXIAL_SPEED = 0.5;
    final double LATERAL_SPEED = 0.6;
    final double YAW_SPEED = 0.5;
    final double SLOW_MODE_MULTIPLIER = 0.4;

    // Roller
    final double ROLLER_FLAT = 0.95;
    final double ROLLER_UPSIDEDOWN = 0.275;

    // Arm
    final double ARM_EXTEND_SPEED = 1;
    final double ARM_LIFT_SPEED = 3;
    final double ARM_LIFT_POWER = 0.4;
    final int ARM_MAX_POSITION = 3050;

    // Grip
    final double GRIP_OPEN = 0.5;
    final double GRIP_CLOSED = 0;
    final double GRIP_TRIGGER_THRESHOLD = 0.1;

    // Startup Sequence
    final int ARM_LIFT_POSITION = 500;
    final int ROLLER_WAIT_TIME = 1000;

    // Pixel Pickup Sequence
    final int PICKUP_EXTEND_TARGET = 1200;

    // Pixel Backdrop Sequence
    final int BACKDROP_EXTEND_TARGET = 1200;

    ElapsedTime runtime = new ElapsedTime();

    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;
    DcMotor armLift = null;
    DcMotor armExtend = null;
    Servo rightGrip = null;
    Servo leftGrip = null;
    Servo roller = null;

    DigitalChannel digital1 = null;
    DigitalChannel digital0 = null;

    int currentArmLiftPos = 0;
    boolean slowModeActive = false;

    public void Movement() {
        double max;

        double axialMultiplier = AXIAL_SPEED * (slowModeActive ? SLOW_MODE_MULTIPLIER : 1);
        double lateralMultiplier = LATERAL_SPEED * (slowModeActive ? SLOW_MODE_MULTIPLIER : 1);
        double yawMultiplier = YAW_SPEED * (slowModeActive ? SLOW_MODE_MULTIPLIER : 1);

        double axial = gamepad1.left_stick_y * axialMultiplier;
        double lateral = -gamepad1.left_stick_x * lateralMultiplier;
        double yaw = gamepad1.right_stick_x * yawMultiplier;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to wheels
        frontLeftMotor.setPower(leftFrontPower);
        frontRightMotor.setPower(rightFrontPower);
        backLeftMotor.setPower(leftBackPower);
        backRightMotor.setPower(rightBackPower);
    }

    public void StartupSequence() {
        armLift.setTargetPosition(ARM_LIFT_POSITION);
        armExtend.setPower(ARM_EXTEND_SPEED);

        while (armLift.isBusy()) {}

        armExtend.setPower(0);
        roller.setPosition(ROLLER_FLAT);
        leftGrip.setPosition(GRIP_OPEN);
        rightGrip.setPosition(GRIP_OPEN);

        sleep(ROLLER_WAIT_TIME);

        armLift.setTargetPosition(0);
    }

    public void PixelPickupSequence() {
        armLift.setTargetPosition(0);
        leftGrip.setPosition(GRIP_OPEN);
        rightGrip.setPosition(GRIP_OPEN);
        roller.setPosition(ROLLER_FLAT);

        if (armExtend.getCurrentPosition() < PICKUP_EXTEND_TARGET) {
            armExtend.setPower(-ARM_EXTEND_SPEED);
            while (armExtend.getCurrentPosition() < PICKUP_EXTEND_TARGET) {}
        }
        else if (armExtend.getCurrentPosition() > PICKUP_EXTEND_TARGET) {
            armExtend.setPower(ARM_EXTEND_SPEED);
            while (armExtend.getCurrentPosition() > PICKUP_EXTEND_TARGET) {}
        }

        armExtend.setPower(0);
    }

    public void PixelBackdropSequence() {
        armLift.setTargetPosition(ARM_MAX_POSITION);
        roller.setPosition(ROLLER_UPSIDEDOWN);

        if (armExtend.getCurrentPosition() < BACKDROP_EXTEND_TARGET) {
            armExtend.setPower(-ARM_EXTEND_SPEED);
            while (armExtend.getCurrentPosition() < BACKDROP_EXTEND_TARGET) {}
        }
        else if (armExtend.getCurrentPosition() > BACKDROP_EXTEND_TARGET) {
            armExtend.setPower(ARM_EXTEND_SPEED);
            while (armExtend.getCurrentPosition() > BACKDROP_EXTEND_TARGET) {}
        }

        armExtend.setPower(0);
    }

    @Override
    public void runOpMode() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "motor0");
        backLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        frontRightMotor = hardwareMap.get(DcMotor.class, "motor2");
        backRightMotor = hardwareMap.get(DcMotor.class, "motor3");
        //frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        //backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        //frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        //backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");

        armLift = hardwareMap.dcMotor.get("armLift");
        armExtend = hardwareMap.dcMotor.get("armExtend");
        rightGrip = hardwareMap.servo.get("GripR");
        leftGrip = hardwareMap.servo.get("GripL");
        roller = hardwareMap.servo.get("Roll");
        digital1 = hardwareMap.get(DigitalChannel.class, "digital1");
        digital0 = hardwareMap.get(DigitalChannel.class, "digital0");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        armExtend.setDirection(DcMotor.Direction.REVERSE);
        armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightGrip.setDirection(Servo.Direction.REVERSE);

        armLift.setTargetPosition(0);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setPower(ARM_LIFT_POWER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        StartupSequence();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Movement
            slowModeActive = gamepad1.right_bumper;

            Movement();

            // Roller
            if (gamepad2.x) {
                roller.setPosition(ROLLER_UPSIDEDOWN);
            }

            if (gamepad2.b) {
                roller.setPosition(ROLLER_FLAT);
            }

            // Arm
            armExtend.setPower(gamepad2.right_stick_y * ARM_EXTEND_SPEED);

            currentArmLiftPos -= (int)(gamepad2.left_stick_y * ARM_LIFT_SPEED);
            if (currentArmLiftPos < 0) currentArmLiftPos = 0;
            if (currentArmLiftPos > ARM_MAX_POSITION) currentArmLiftPos = ARM_MAX_POSITION;

            armLift.setTargetPosition(currentArmLiftPos);

            // Grip
            if (gamepad2.left_bumper) {
                leftGrip.setPosition(GRIP_OPEN);
            }

            if (gamepad2.right_bumper) {
                rightGrip.setPosition(GRIP_OPEN);
            }

            if (gamepad2.left_trigger > GRIP_TRIGGER_THRESHOLD) {
                leftGrip.setPosition(GRIP_CLOSED);
            }

            if (gamepad2.right_trigger > GRIP_TRIGGER_THRESHOLD) {
                rightGrip.setPosition(GRIP_CLOSED);
            }

            // Sequences
            if (gamepad2.a) {
                PixelPickupSequence();
            }

            if (gamepad2.y) {
                PixelBackdropSequence();
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("arm extend position", armExtend.getCurrentPosition());
            telemetry.update();
        }
    }
}
