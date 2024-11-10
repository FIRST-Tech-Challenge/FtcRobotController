package com.kalipsorobotics.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.kalipsorobotics.utilities.OpModeUtilities;

public class Intake {
    private final OpModeUtilities opModeUtilities;

    private DcMotor noodleMotor;
    private Servo pivotServo, sampleDoor;

    public Intake(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;

        setUpHardware();
    }

    private void setUpHardware() {
        noodleMotor = opModeUtilities.getHardwareMap().dcMotor.get("intakeNoodleMotor");
        pivotServo = opModeUtilities.getHardwareMap().servo.get("intakePivotServo");
        sampleDoor = opModeUtilities.getHardwareMap().servo.get("sampleDoorServo");

        noodleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        noodleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }

    public DcMotor getNoodleMotor() {
        return noodleMotor;
    }

    public Servo getPivotServo() {
        return pivotServo;
    }

    public Servo getSampleDoor() {
        return sampleDoor;
    }
}
