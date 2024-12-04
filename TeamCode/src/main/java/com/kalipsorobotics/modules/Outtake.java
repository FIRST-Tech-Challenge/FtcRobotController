package com.kalipsorobotics.modules;

import com.kalipsorobotics.utilities.KServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.kalipsorobotics.utilities.OpModeUtilities;

public class Outtake {

    //TODO MAKE PRIVATE
    private final OpModeUtilities opModeUtilities;
    public DcMotor linearSlide1, linearSlide2;
    public KServo outtakePivotServo;
    public KServo outtakeClawServo;
    public KServo outtakePigeonServo;
    public KServo hangHook1;
    public KServo hangHook2;

    public Outtake(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        setUpHardware();
    }

    private void setUpHardware() {
        linearSlide1 = opModeUtilities.getHardwareMap().dcMotor.get("linearSlide1");
        linearSlide2 = opModeUtilities.getHardwareMap().dcMotor.get("linearSlide2");
        outtakePivotServo = new KServo(opModeUtilities.getHardwareMap().servo.get("outtakePivotServo"), 60/0.11, 300,
                0, false);
        outtakeClawServo = new KServo(opModeUtilities.getHardwareMap().servo.get("clawServo"), 60/0.25, 300,
                0, false);
        outtakePigeonServo = new KServo(opModeUtilities.getHardwareMap().servo.get("outtakePigeonServo"), 60/0.25, 300,
                0, false);
        hangHook1 = new KServo(opModeUtilities.getHardwareMap().servo.get("hang1"), 60/0.25, 300,
                0, false);
        hangHook2 = new KServo(opModeUtilities.getHardwareMap().servo.get("hang2"), 60/0.25, 300,
                0, false);

        linearSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linearSlide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearSlide1.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlide2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public DcMotor getLinearSlide1() {
        return linearSlide1;
    }
    public DcMotor getLinearSlide2() {
        return linearSlide2;
    }
    public KServo getOuttakePivotServo() {
        return outtakePivotServo;
    }
    public KServo getOuttakeClawServo() {
        return outtakeClawServo;
    }
    public KServo getOuttakePigeonServo() {
        return outtakePigeonServo;
    }
    public KServo getHangHook1() {return hangHook1;}

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }
}
