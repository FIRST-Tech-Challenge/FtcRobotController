package com.kalipsorobotics.modules;

import com.kalipsorobotics.utilities.KServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.kalipsorobotics.utilities.OpModeUtilities;

public class Intake {
    private final OpModeUtilities opModeUtilities;

    private DcMotor noodleMotor;
    private KServo intakePivotServo;
    private KServo doorServo;
    //private KServo linkageServo1, linkageServo2;

    private DcMotor linkageMotor;
    private ColorSensor colorSensor;


    public Intake(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        setUpHardware();
    }

    private void setUpHardware() {
        noodleMotor = opModeUtilities.getHardwareMap().dcMotor.get("intakeNoodleMotor");
        intakePivotServo = new KServo(opModeUtilities.getHardwareMap().servo.get("intakePivotServo"), 60/0.25, 300,
                0, false);
        doorServo = new KServo(opModeUtilities.getHardwareMap().servo.get("doorServo"), 60/0.25, 300,
                0, false);
/*        linkageServo1 = new KServo(opModeUtilities.getHardwareMap().servo.get("linkageServo1"), 60/0.25, 300,
                0, false);
        linkageServo2 = new KServo(opModeUtilities.getHardwareMap().servo.get("linkageServo2"), 60/0.25, 300,
                0, false);*/
        linkageMotor = opModeUtilities.getHardwareMap().dcMotor.get("intakeLinkage");
        colorSensor = opModeUtilities.getHardwareMap().colorSensor.get("intakeColorSensor");

        noodleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        noodleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linkageMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linkageMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        linkageMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkageMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }

    public DcMotor getNoodleMotor() {
        return noodleMotor;
    }

    public KServo getIntakePivotServo() {
        return intakePivotServo;
    }

    public KServo getDoorServo() {
        return doorServo;
    }

    /*public KServo getLinkageServo1() {
        return linkageServo1;
    }

    public KServo getLinkageServo2() {
        return linkageServo2;
    }*/

    public DcMotor getLinkageMotor() {
        return linkageMotor;
    }

    public ColorSensor getColorSensor() {
        return colorSensor;
    }
}

