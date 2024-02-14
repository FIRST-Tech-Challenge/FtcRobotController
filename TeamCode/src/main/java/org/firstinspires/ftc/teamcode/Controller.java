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
    // movement
    final double AXIAL_SPEED = 0.5;
    final double LATERAL_SPEED = 0.6;
    final double YAW_SPEED = 0.5;
    final double SLOW_MODE_MULTIPLIER = 0.4;

    // Roller
    final double ROLLER_FLAT = 0.29;
    final double ROLLER_UPSIDEDOWN = 0.97;
    final double ROLLER_ARM_LIMIT = 400;

    // Arm
    final double ARM_EXTEND_SPEED = 0.5;
    final double ARM_LIFT_SPEED = 7;
    final double ARM_LIFT_POWER = 0.4;
    final int ARM_MAX_POSITION = 3050;
    final int ARM_EXTEND_LIMIT = 1000;

    // Grip
    final double GRIP_OPEN = 0.5;
    final double GRIP_CLOSED = 0;
    final double GRIP_TRIGGER_THRESHOLD = 0.1;

    // Startup Sequence
    final int ARM_LIFT_POSITION = 600;
    final int ROLLER_WAIT_TIME = 1000;

    // Pixel Pickup Sequence
    final int PICKUP_EXTEND_TARGET = 800;

    // Pixel Backdrop Sequence
    final int BACKDROP_EXTEND_TARGET = 800;

    // Find Arm Extend Zero
    final double ARM_EXTEND_FAST_SPEED = 0.1;
    final double ARM_EXTEND_SLOW_SPEED = 0.01;

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

    DigitalChannel armExtendSwitch = null;
    DigitalChannel digital1 = null;

    int armExtendZero = 0;
    int currentArmLiftPos = 0;
    boolean slowModeActive = false;
    boolean pickupSequenceActive = false;
    boolean backdropSequenceActive = false;
    boolean overrideMode = false;

    public void movement() {
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

    public void findArmExtendZero() {
        if (armExtendSwitch.getState()) { // If switch is not pressed
            armExtend.setPower(-ARM_EXTEND_SLOW_SPEED);

            telemetry.addData("test", 1);
            telemetry.update();

            while (armExtendSwitch.getState()) {} // While not pressed

            armExtend.setPower(0);
        }

        // Go forward slowly and then backwards and find the switch
        armExtend.setPower(ARM_EXTEND_SLOW_SPEED);
        telemetry.addData("test", 2);
        telemetry.update();
        while (!armExtendSwitch.getState()) {} // While pressed

        armExtend.setPower(-ARM_EXTEND_SLOW_SPEED);
        telemetry.addData("test", 3);
        telemetry.update();
        while (armExtendSwitch.getState()) {} // While not pressed
        telemetry.addData("test", 4);
        telemetry.update();
        armExtend.setPower(0);
        armExtendZero = armExtend.getCurrentPosition();
    }

    public void startupSequence() {
        currentArmLiftPos = ARM_LIFT_POSITION;
        armLift.setTargetPosition(currentArmLiftPos);
        armExtend.setPower(-ARM_EXTEND_SPEED);

        while (armLift.isBusy()) {}

        armExtend.setPower(0);

        findArmExtendZero();

        roller.setPosition(ROLLER_FLAT);
        leftGrip.setPosition(GRIP_OPEN);
        rightGrip.setPosition(GRIP_OPEN);

        sleep(ROLLER_WAIT_TIME);

        currentArmLiftPos = 0;
        armLift.setTargetPosition(0);
    }

    public void pixelPickupSequence() {
        pickupSequenceActive = true;

        currentArmLiftPos = 0;
        armLift.setTargetPosition(currentArmLiftPos);
        leftGrip.setPosition(GRIP_OPEN);
        rightGrip.setPosition(GRIP_OPEN);
        roller.setPosition(ROLLER_FLAT);

        if (armExtend.getCurrentPosition() < PICKUP_EXTEND_TARGET) {
            armExtend.setPower(ARM_EXTEND_SPEED);
            while (armExtend.getCurrentPosition() < PICKUP_EXTEND_TARGET) {}
        }
        else if (armExtend.getCurrentPosition() > PICKUP_EXTEND_TARGET) {
            armExtend.setPower(-ARM_EXTEND_SPEED);
            while (armExtend.getCurrentPosition() > PICKUP_EXTEND_TARGET) {}
        }

        armExtend.setPower(0);

        pickupSequenceActive = false;
    }

    public void pixelBackdropSequence() {
        backdropSequenceActive = true;

        currentArmLiftPos = ARM_MAX_POSITION;
        armLift.setTargetPosition(currentArmLiftPos);
        roller.setPosition(ROLLER_UPSIDEDOWN);

        if (armExtend.getCurrentPosition() < BACKDROP_EXTEND_TARGET) {
            armExtend.setPower(ARM_EXTEND_SPEED);
            while (armExtend.getCurrentPosition() < BACKDROP_EXTEND_TARGET) {}
        }
        else if (armExtend.getCurrentPosition() > BACKDROP_EXTEND_TARGET) {
            armExtend.setPower(-ARM_EXTEND_SPEED);
            while (armExtend.getCurrentPosition() > BACKDROP_EXTEND_TARGET) {}
        }

        armExtend.setPower(0);

        backdropSequenceActive = false;
    }
    
    public void interrupt() {
        while (true) {
            if (!overrideMode) {
                if (!armExtendSwitch.getState() && armExtend.getPower() < 0) { // Pressed and retracting
                    armExtend.setPower(0);
                }
            }

            sleep(1);
        }
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
        armExtendSwitch = hardwareMap.get(DigitalChannel.class, "digital0");
        digital1 = hardwareMap.get(DigitalChannel.class, "digital1");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rightGrip.setDirection(Servo.Direction.REVERSE);
        armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setTargetPosition(0);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setPower(ARM_LIFT_POWER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        new Thread(this::interrupt).start();
        startupSequence();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // movement
            slowModeActive = gamepad1.right_bumper;

            movement();

            // Roller
            if (currentArmLiftPos >= ROLLER_ARM_LIMIT || overrideMode) {
                if (gamepad2.x) {
                    roller.setPosition(ROLLER_UPSIDEDOWN);
                }

                if (gamepad2.b) {
                    roller.setPosition(ROLLER_FLAT);
                }
            }

            // Arm Extend
            if (armExtend.getCurrentPosition() > armExtendZero + 10 || armExtend.getCurrentPosition() < ARM_EXTEND_LIMIT || -gamepad2.right_stick_y > 0 || armExtendSwitch.getState() || overrideMode) {
                armExtend.setPower(-gamepad2.right_stick_y * ARM_EXTEND_SPEED);
            }

            // Arm Lift
            currentArmLiftPos -= (int)(gamepad2.left_stick_y * ARM_LIFT_SPEED);
            if (currentArmLiftPos < 0) currentArmLiftPos = 0;
            if (currentArmLiftPos > ARM_MAX_POSITION && !overrideMode) currentArmLiftPos = ARM_MAX_POSITION;

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
            if (gamepad2.a && !pickupSequenceActive) {
                new Thread(this::pixelPickupSequence).start();
            }

            if (gamepad2.y && !backdropSequenceActive) {
                new Thread(this::pixelBackdropSequence).start();
            }

            // Override Mode
            overrideMode = gamepad2.dpad_down;

            // Debugging
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("arm extend position", armExtend.getCurrentPosition());
            telemetry.addData("arm extend zero", armExtendZero);
            telemetry.addData("arm extend switch", armExtendSwitch.getState());
            telemetry.update();
        }
    }
}
