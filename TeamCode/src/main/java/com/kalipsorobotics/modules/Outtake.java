package com.kalipsorobotics.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.kalipsorobotics.utilities.OpModeUtilities;

public class Outtake {
    private final OpModeUtilities opModeUtilities;
    public static final double P_CONSTANT = 0.004;

    public DcMotor linearSlideMotor1, linearSlideMotor2;
    public Servo outtakePivotServo;
    public Servo clawServo;
    public Servo outtakePigeonServo;

    public Outtake(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        setUpHardware();
    }

    private void setUpHardware() {
        linearSlideMotor1 = opModeUtilities.getHardwareMap().dcMotor.get("linearSlide1");
        linearSlideMotor2 = opModeUtilities.getHardwareMap().dcMotor.get("linearSlide2");
        outtakePivotServo = opModeUtilities.getHardwareMap().servo.get("outtakePivotServo");
        clawServo = opModeUtilities.getHardwareMap().servo.get("clawServo");
        outtakePigeonServo = opModeUtilities.getHardwareMap().servo.get("outtakePigeonServo");

        linearSlideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linearSlideMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlideMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public DcMotor getLinearSlide2() {
        return linearSlideMotor2;
    }
    public Servo getOuttakePivotServo() {
        return outtakePivotServo;
    }
    public Servo getClaw() {
        return clawServo;
    }
    public Servo getOuttakePigeonServo() {
        return outtakePigeonServo;
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }
}
