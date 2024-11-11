package com.kalipsorobotics.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.kalipsorobotics.utilities.OpModeUtilities;

public class Outtake {
    private final OpModeUtilities opModeUtilities;
    public static final double P_CONSTANT = 0.004;

    public DcMotor linearSlide1, linearSlide2;
    public Servo armPivot;
    public Servo claw;
    //public Servo pigeonHead;

    public Outtake(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        setUpHardware();
    }

    private void setUpHardware() {
        linearSlide1 = opModeUtilities.getHardwareMap().dcMotor.get("linearSlide1");
        linearSlide2 = opModeUtilities.getHardwareMap().dcMotor.get("linearSlide2");
        armPivot = opModeUtilities.getHardwareMap().servo.get("armPivot");
        claw = opModeUtilities.getHardwareMap().servo.get("claw");
        //pigeonHead = opModeUtilities.getHardwareMap().servo.get("pigeonHead");

        linearSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }
}
