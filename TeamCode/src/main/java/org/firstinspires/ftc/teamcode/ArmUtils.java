package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmUtils {
    // Roller
    final double ROLLER_FLAT = 0.29;
    final double ROLLER_UPSIDEDOWN = 0.97;
    final double ROLLER_ARM_LIMIT = 400;

    // Arm
    final double ARM_EXTEND_SPEED = 0.5;
    final double ARM_LIFT_SPEED = 6;
    final double ARM_LIFT_POWER = 0.6;
    final int ARM_MAX_POSITION = 3200;
    final int ARM_MIN_POSITION = -100;
    final int ARM_EXTEND_LIMIT = 3600;

    // Grip
    final double GRIP_OPEN = 0.5;
    final double GRIP_CLOSED = 0;
    final double GRIP_TRIGGER_THRESHOLD = 0.1;

    // Startup Sequence
    final int ARM_LIFT_POSITION = 800;
    //final int ROLLER_WAIT_TIME = 1000;

    // Pixel Pickup Sequence
    final int PICKUP_EXTEND_TARGET = 100;

    // Pixel Backdrop Sequence
    final int BACKDROP_EXTEND_TARGET = 2000;
    final int BACKDROP_ARM_TARGET = 3000;

    final double SEQUENCE_ARM_POWER = 0.5;

    Controller controller;

    DcMotor armLift = null;
    DcMotor armExtend = null;
    Servo rightGrip = null;
    Servo leftGrip = null;
    Servo rollerServo = null;

    int currentArmLiftPos = 0;
    boolean startupSequenceActive = false;
    boolean pickupSequenceActive = false;
    boolean backdropSequenceActive = false;

    public ArmUtils(Controller controller, HardwareMap hardwareMap) {
        this.controller = controller;

        armLift = hardwareMap.dcMotor.get("armLift");
        armExtend = hardwareMap.dcMotor.get("armExtend");
        rightGrip = hardwareMap.servo.get("GripR");
        leftGrip = hardwareMap.servo.get("GripL");
        rollerServo = hardwareMap.servo.get("Roll");

        rightGrip.setDirection(Servo.Direction.REVERSE);
        armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setTargetPosition(0);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setPower(ARM_LIFT_POWER);
    }

    public void startupSequence() {
        startupSequenceActive = true;

        leftGrip.setPosition(GRIP_CLOSED);
        rightGrip.setPosition(GRIP_CLOSED);
        armLift.setPower(SEQUENCE_ARM_POWER);
        currentArmLiftPos = ARM_LIFT_POSITION;
        armLift.setTargetPosition(currentArmLiftPos);
        armExtend.setPower(-ARM_EXTEND_SPEED);

        if (!armLift.isBusy()) {
            armLift.setPower(ARM_LIFT_POWER);
            armExtend.setPower(0);

            rollerServo.setPosition(ROLLER_FLAT);

            startupSequenceActive = false;
        }

        //sleep(ROLLER_WAIT_TIME);

        //currentArmLiftPos = 0;
        //armLift.setTargetPosition(currentArmLiftPos);
    }

    void pixelPickupSequence() {
        pickupSequenceActive = true;

        armLift.setPower(SEQUENCE_ARM_POWER);
        currentArmLiftPos = ARM_MIN_POSITION;
        armLift.setTargetPosition(currentArmLiftPos);
        leftGrip.setPosition(GRIP_OPEN);
        rightGrip.setPosition(GRIP_OPEN);
        rollerServo.setPosition(ROLLER_FLAT);

        controller.Debug("arm extend", -armExtend.getCurrentPosition());

        if (-armExtend.getCurrentPosition() < PICKUP_EXTEND_TARGET) {
            armExtend.setPower(ARM_EXTEND_SPEED);
        }
        else if (-armExtend.getCurrentPosition() > PICKUP_EXTEND_TARGET) {
            armExtend.setPower(-ARM_EXTEND_SPEED);
        }

        if (!armLift.isBusy()) {
            armExtend.setPower(0);
            armLift.setPower(ARM_LIFT_POWER);

            pickupSequenceActive = false;
        }
    }

    void pixelBackdropSequence() {
        backdropSequenceActive = true;

        armLift.setPower(SEQUENCE_ARM_POWER);
        currentArmLiftPos = BACKDROP_ARM_TARGET;
        armLift.setTargetPosition(currentArmLiftPos);
        rollerServo.setPosition(ROLLER_UPSIDEDOWN);

        if (-armExtend.getCurrentPosition() < BACKDROP_EXTEND_TARGET) {
            armExtend.setPower(ARM_EXTEND_SPEED);
        }
        else if (-armExtend.getCurrentPosition() > BACKDROP_EXTEND_TARGET) {
            armExtend.setPower(-ARM_EXTEND_SPEED);
        }

        if (!armLift.isBusy()) {
            armExtend.setPower(0);
            armLift.setPower(ARM_LIFT_POWER);

            backdropSequenceActive = false;
        }
    }

    public void runSequences(Gamepad gamepad) {
        if (!startupSequenceActive && !backdropSequenceActive && !pickupSequenceActive) {
            if (gamepad.a) {
                pixelPickupSequence();
            }
            else if (gamepad.y) {
                pixelBackdropSequence();
            }
        }

        if (startupSequenceActive) {
            startupSequence();
        }
        else if (pickupSequenceActive) {
            pixelPickupSequence();
        }
        else if (backdropSequenceActive) {
            pixelBackdropSequence();
        }
    }

    public void roller(Gamepad gamepad) {
        if (startupSequenceActive || pickupSequenceActive || backdropSequenceActive) return;

        boolean overrideMode = gamepad.dpad_down;

        if (currentArmLiftPos >= ROLLER_ARM_LIMIT || overrideMode) {
            if (gamepad.x) {
                rollerServo.setPosition(ROLLER_UPSIDEDOWN);
            }

            if (gamepad.b) {
                rollerServo.setPosition(ROLLER_FLAT);
            }
        }
    }

    public void extend(Gamepad gamepad) {
        if (startupSequenceActive || pickupSequenceActive || backdropSequenceActive) return;

        boolean overrideMode = gamepad.dpad_down;

        if (-armExtend.getCurrentPosition() < ARM_EXTEND_LIMIT || overrideMode || -gamepad.right_stick_y < 0) {
            armExtend.setPower(-gamepad.right_stick_y * ARM_EXTEND_SPEED);
        }
        else armExtend.setPower(0);
    }

    public void lift(Gamepad gamepad) {
        if (startupSequenceActive || pickupSequenceActive || backdropSequenceActive) return;

        boolean overrideMode = gamepad.dpad_down;

        currentArmLiftPos -= (int)(gamepad.left_stick_y * ARM_LIFT_SPEED);
        if (currentArmLiftPos < ARM_MIN_POSITION && !overrideMode && -gamepad.left_stick_y < 0) currentArmLiftPos = ARM_MIN_POSITION;
        if (currentArmLiftPos > ARM_MAX_POSITION && !overrideMode && -gamepad.left_stick_y > 0) currentArmLiftPos = ARM_MAX_POSITION;

        armLift.setTargetPosition(currentArmLiftPos);
    }

    public void grip(Gamepad gamepad) {
        if (startupSequenceActive || pickupSequenceActive || backdropSequenceActive) return;

        if (gamepad.left_bumper) {
            leftGrip.setPosition(GRIP_OPEN);
        }

        if (gamepad.right_bumper) {
            rightGrip.setPosition(GRIP_OPEN);
        }

        if (gamepad.left_trigger > GRIP_TRIGGER_THRESHOLD) {
            leftGrip.setPosition(GRIP_CLOSED);
        }

        if (gamepad.right_trigger > GRIP_TRIGGER_THRESHOLD) {
            rightGrip.setPosition(GRIP_CLOSED);
        }
    }
}
