package com.kalipsorobotics.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.kalipsorobotics.utilities.OpModeUtilities;

public class Outtake2024 {
    private final OpModeUtilities opModeUtilities;
    public static final double P_CONSTANT = 0.004;

    public DcMotor linearSlide, linearSlideTwo;
    public Servo armPivot, claw;

    public Outtake2024(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        setUpHardware();
    }

    private void setUpHardware() {
        linearSlide = opModeUtilities.getHardwareMap().dcMotor.get("linearSlide");
        linearSlideTwo = opModeUtilities.getHardwareMap().dcMotor.get("linearSlideTwo");
        armPivot = opModeUtilities.getHardwareMap().servo.get("armPivot");
        claw = opModeUtilities.getHardwareMap().servo.get("claw");

        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }
}
