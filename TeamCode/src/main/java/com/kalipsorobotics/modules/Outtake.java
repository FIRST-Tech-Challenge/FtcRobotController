package com.kalipsorobotics.modules;

import com.kalipsorobotics.utilities.KServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.kalipsorobotics.utilities.OpModeUtilities;

public class Outtake {
    private final OpModeUtilities opModeUtilities;
    public static final double P_CONSTANT = 0.004;
    public static final double LS_STAYUP_POWER = 0.1;

    public DcMotor linearSlideMotor1, linearSlideMotor2;
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
        linearSlideMotor1 = opModeUtilities.getHardwareMap().dcMotor.get("linearSlide1");
        linearSlideMotor2 = opModeUtilities.getHardwareMap().dcMotor.get("linearSlide2");
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

        linearSlideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linearSlideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearSlideMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlideMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public DcMotor getLinearSlideMotor1() {
        return linearSlideMotor1;
    }
    public DcMotor getLinearSlide2() {
        return linearSlideMotor2;
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
