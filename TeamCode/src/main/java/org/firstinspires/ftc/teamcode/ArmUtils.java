package org.firstinspires.ftc.teamcode;

//import static java.lang.Thread.sleep;


import static android.os.SystemClock.sleep;

import android.icu.text.Transliterator;

import com.qualcomm.robotcore.hardware.*;

public class ArmUtils {
    // Roller
    final double ROLLER_FLAT = 0.29;
    final double ROLLER_UPSIDEDOWN = 0.97;
    final double ROLLER_ARM_LIMIT = 400;

    // Arm
    final double ARM_EXTEND_SPEED = 0.5;
    final double ARM_LIFT_SPEED = 40;
    final double ARM_LIFT_POWER = 0.4;
    final int ARM_MAX_POSITION = 3200;
    final int ARM_MIN_POSITION = -100;
    final int ARM_EXTEND_LIMIT = 3600;

    // Grip
    final double GRIP_OPEN = 0.5;
    final double GRIP_CLOSED = 0.2;
    final double GRIP_TRIGGER_THRESHOLD = 0.1;

    // Startup Sequence
    final int ARM_LIFT_POSITION = 800;
    //final int ROLLER_WAIT_TIME = 1000;

    // Pixel Pickup Sequence
    final int PICKUP_EXTEND_TARGET = 1700;

    // Pixel Reverse Backdrop Sequence
    final int REVERSE_BACKDROP_EXTEND_TARGET = 2000;
    final int REVERSE_BACKDROP_ARM_TARGET = 3000;

    // Pixel Forward Backdrop Sequence
    final int FORWARD_BACKDROP_EXTEND_TARGET = 2300;
    final int FORWARD_BACKDROP_ARM_TARGET = 1000;

    // All Sequences
    final double SEQUENCE_ARM_POWER = 0.5;

    // Drone
    final double DRONE_SHOOT = 0.3;

    Controller controller;

    DcMotor armLift = null;
    DcMotor armExtend = null;
    Servo rightGrip = null;
    Servo leftGrip = null;
    Servo rollerServo = null;
    Servo droneServo = null;

    int currentArmLiftPos = 0;

    boolean startupSequenceActive = false;
    boolean pickupSequenceActive = false;
    boolean backdropReverseSequenceActive = false;
    boolean backdropForwardSequenceActive = false;
    boolean sequenceActive = false;

    ExtendDirection sequenceDirection = ExtendDirection.UNINITIALIZED;
    boolean sequenceGotToPosition = false;

    public ArmUtils(Controller controller, HardwareMap hardwareMap) {
        this.controller = controller;

        armLift = hardwareMap.dcMotor.get("armLift");
        armExtend = hardwareMap.dcMotor.get("armExtend");
        rightGrip = hardwareMap.servo.get("gripR");
        leftGrip = hardwareMap.servo.get("gripL");
        rollerServo = hardwareMap.servo.get("roll");
        droneServo = hardwareMap.servo.get("droneServo");

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
        pickupSequenceActive = baseSequence(ARM_MIN_POSITION, PICKUP_EXTEND_TARGET, ROLLER_FLAT, GRIP_OPEN);
    }

    void pixelReverseBackdropSequence() {
        backdropReverseSequenceActive = baseSequence(REVERSE_BACKDROP_ARM_TARGET, REVERSE_BACKDROP_EXTEND_TARGET, ROLLER_UPSIDEDOWN, leftGrip.getPosition());
    }

    void pixelForwardBackdropSequence() {
        backdropForwardSequenceActive = baseSequence(FORWARD_BACKDROP_ARM_TARGET, FORWARD_BACKDROP_EXTEND_TARGET, ROLLER_FLAT, leftGrip.getPosition());
    }

    boolean baseSequence(int armTarget, int extendTarget, double rollerTarget, double gripTarget) {
        armLift.setPower(SEQUENCE_ARM_POWER);
        currentArmLiftPos = armTarget;
        armLift.setTargetPosition(currentArmLiftPos);
        leftGrip.setPosition(gripTarget);
        rightGrip.setPosition(gripTarget);
        rollerServo.setPosition(rollerTarget);

        if (!sequenceGotToPosition) {
            if (-armExtend.getCurrentPosition() < extendTarget && sequenceDirection != ExtendDirection.BACKWARD) {
                sequenceDirection = ExtendDirection.FORWARD;
                armExtend.setPower(ARM_EXTEND_SPEED);
            }
            else if (-armExtend.getCurrentPosition() > extendTarget && sequenceDirection != ExtendDirection.FORWARD) {
                sequenceDirection = ExtendDirection.BACKWARD;
                armExtend.setPower(-ARM_EXTEND_SPEED);
            }

            sequenceGotToPosition = (sequenceDirection == ExtendDirection.FORWARD && -armExtend.getCurrentPosition() >= extendTarget) || (sequenceDirection == ExtendDirection.BACKWARD && -armExtend.getCurrentPosition() <= extendTarget);
        }
        else {
            armExtend.setPower(0);
        }

        if (!armLift.isBusy() && sequenceGotToPosition) {
            sequenceDirection = ExtendDirection.UNINITIALIZED;
            sequenceGotToPosition = false;

            armExtend.setPower(0);
            armLift.setPower(ARM_LIFT_POWER);

            return false;
        }

        return true;
    }

    public void runSequences(Gamepad gamepad) {
        sequenceActive = startupSequenceActive || backdropReverseSequenceActive || backdropForwardSequenceActive || pickupSequenceActive;

        if (!sequenceActive) {
            if (gamepad.a) {
                pixelPickupSequence();
            }
            else if (gamepad.y) {
                //pixelReverseBackdropSequence();
                pixelForwardBackdropSequence();
            }
        }

        if (startupSequenceActive) {
            startupSequence();
        }
        else if (pickupSequenceActive) {
            pixelPickupSequence();
        }
        else if (backdropForwardSequenceActive) {
            pixelForwardBackdropSequence();
        }
        else if (backdropReverseSequenceActive) {
            pixelReverseBackdropSequence();
        }
    }

    public void roller(Gamepad gamepad) {
        if (sequenceActive) return;

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
        if (sequenceActive) return;

        boolean overrideMode = gamepad.dpad_down;

        if (-armExtend.getCurrentPosition() < ARM_EXTEND_LIMIT || overrideMode || -gamepad.right_stick_y < 0) {
            armExtend.setPower(-gamepad.right_stick_y * ARM_EXTEND_SPEED);
        }
        else armExtend.setPower(0);
    }

    public void lift(Gamepad gamepad) {
        if (sequenceActive) return;

        boolean overrideMode = gamepad.dpad_down;

        currentArmLiftPos -= (int)(gamepad.left_stick_y * ARM_LIFT_SPEED);
        if (currentArmLiftPos < ARM_MIN_POSITION && !overrideMode && -gamepad.left_stick_y < 0) currentArmLiftPos = ARM_MIN_POSITION;
        if (currentArmLiftPos > ARM_MAX_POSITION && !overrideMode && -gamepad.left_stick_y > 0) currentArmLiftPos = ARM_MAX_POSITION;

        armLift.setTargetPosition(currentArmLiftPos);
    }

    public void grip(Gamepad gamepad) {
        if (sequenceActive) return;

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

    public void drone(Gamepad gamepad) {
        if (gamepad.guide) {
            droneServo.setPosition(DRONE_SHOOT);
            sleep(100);
            droneServo.getController().pwmDisable();
        }
    }
}

enum ExtendDirection {
    UNINITIALIZED,
    FORWARD,
    BACKWARD
}
