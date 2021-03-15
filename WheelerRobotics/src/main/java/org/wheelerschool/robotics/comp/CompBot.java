package org.wheelerschool.robotics.comp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CompBot {
    public DcMotor launchLeft;
    public DcMotor launchRight;
    public Servo launchPush;

    public Servo wobbleGrab;
    public DcMotor wobbleArm;

    public DcMotor driveFLeft;
    public DcMotor driveFRight;
    public DcMotor driveBLeft;
    public DcMotor driveBRight;

    public DcMotor intake;

    public enum IntakeMode {
        IN,
        OUT,
        STOP
    }

    public enum WobblePosition {
        GRAB,
        UP,
        STOWED
    }


    public CompBot(HardwareMap hw) {
        launchLeft = hw.dcMotor.get("launchLeft");
        launchLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        launchLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launchRight = hw.dcMotor.get("launchRight");
        launchRight.setDirection(DcMotorSimple.Direction.FORWARD);
        launchRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launchPush = hw.servo.get("launchPush");

        wobbleGrab = hw.servo.get("wobbleGrab");

        wobbleArm = hw.dcMotor.get("wobbleArm");
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleArm.setDirection(DcMotorSimple.Direction.FORWARD);

        driveFLeft  = hw.dcMotor.get("driveFLeft");
        driveFLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        driveFRight = hw.dcMotor.get("driveFRight");
        driveFRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFRight.setDirection(DcMotorSimple.Direction.FORWARD);

        driveBLeft  = hw.dcMotor.get("driveBLeft");
        driveBLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        driveBRight = hw.dcMotor.get("driveBRight");
        driveBRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBRight.setDirection(DcMotorSimple.Direction.FORWARD);

        intake = hw.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setDrive(float fLeft, float fRight, float bLeft, float bRight) {
        driveFLeft.setPower(fLeft);
        driveFRight.setPower(fRight);
        driveBLeft.setPower(bLeft);
        driveBRight.setPower(bRight);
    }

    public void launcher(boolean mode) {
        float power = mode ? 1.0f : 0;

        launchLeft.setPower(power*0.9f);
        launchRight.setPower(power);
    }

    public void launchPush(boolean mode) {
        float pos = mode ? 0.45f : 0.2f;
        launchPush.setPosition(pos);
    }

    public void intakeMode(IntakeMode mode) {
        float power = 0f;

        if (mode == IntakeMode.IN) {
            power = 1.0f;
        } else if (mode == IntakeMode.OUT) {
            power = -0.5f;
        }

        intake.setPower(power);
    }

    public void setWobbleGrab(boolean grab) {
        if (grab) {
            wobbleGrab.setPosition(1f);
        } else {
            wobbleGrab.setPosition(0f);
        }
    }

    public void setWobbleArm(WobblePosition pos) {
        if (pos == WobblePosition.STOWED) {
            wobbleArm.setTargetPosition(-100);
        } else if (pos == WobblePosition.UP) {
            wobbleArm.setTargetPosition(-300);
        } else if (pos == WobblePosition.GRAB) {
            wobbleArm.setTargetPosition(-650);
        }
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArm.setPower(0.5f);
    }

    public void stop() {
        launcher(false);
        setDrive(0,0,0,0);
    }
}
