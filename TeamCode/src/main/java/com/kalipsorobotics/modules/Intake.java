package com.kalipsorobotics.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.kalipsorobotics.utilities.OpModeUtilities;

public class Intake {
    private final OpModeUtilities opModeUtilities;

    private DcMotor noodleMotor;
    private Servo pivotServo;
    private Servo doorServo;
    private Servo linkageServo1, linkageServo2;

    public Intake(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;

        setUpHardware();
    }

    private void setUpHardware() {
        noodleMotor = opModeUtilities.getHardwareMap().dcMotor.get("intakeNoodleMotor");
        pivotServo = opModeUtilities.getHardwareMap().servo.get("intakePivotServo");
        doorServo = opModeUtilities.getHardwareMap().servo.get("doorServo");
        linkageServo1 = opModeUtilities.getHardwareMap().servo.get("linkageServo1");
        linkageServo2 = opModeUtilities.getHardwareMap().servo.get("linkageServo2");


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

    public Servo getDoorServo() {
        return doorServo;
    }

    public Servo getLinkageServo1() {
        return linkageServo1;
    }

    public Servo getLinkageServo2() {
        return linkageServo2;
    }
}

