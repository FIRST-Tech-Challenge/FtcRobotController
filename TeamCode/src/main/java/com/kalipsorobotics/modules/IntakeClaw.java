package com.kalipsorobotics.modules;

import com.kalipsorobotics.utilities.KServo;
import com.kalipsorobotics.utilities.OpModeUtilities;

public class IntakeClaw {

    private final OpModeUtilities opModeUtilities;


    private KServo intakeLinkageServo;

    private KServo intakeBigSweepServo;

    private KServo intakeBigPivotServo;
    private KServo intakeSmallPivotServo;

    private KServo intakeSmallSweepServo;

    private KServo intakeClawServo;


    public IntakeClaw(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        setUpHardware();
    }

    private void setUpHardware() {

        intakeLinkageServo = new KServo(opModeUtilities.getHardwareMap().servo.get("intakeLinkage"), 60/0.25,
                300,
                0, false);

        intakeBigSweepServo = new KServo(opModeUtilities.getHardwareMap().servo.get("intakeBigSweep"), 60/0.25,
                300,
                0, false);

        intakeBigPivotServo = new KServo(opModeUtilities.getHardwareMap().servo.get("intakeBigPivot"), 60/0.25,
                300,
                0, false);

        intakeSmallPivotServo = new KServo(opModeUtilities.getHardwareMap().servo.get("intakeSmallPivot"), 60/0.25,
                300,
                0, false);

        intakeSmallSweepServo = new KServo(opModeUtilities.getHardwareMap().servo.get("intakeSmallSweep"), 60/0.25,
                300,
                0, false);

        intakeClawServo = new KServo(opModeUtilities.getHardwareMap().servo.get("intakeClaw"), 60/0.25,
                300,
                0, false);
    }


    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }


    public KServo getIntakeLinkageServo() {
        return intakeLinkageServo;
    }

    public KServo getIntakeBigSweepServo() {
        return intakeBigSweepServo;
    }

    public KServo getIntakeBigPivotServo() {
        return intakeBigPivotServo;
    }

    public KServo getIntakeSmallPivotServo() {
        return intakeSmallPivotServo;
    }

    public KServo getIntakeSmallSweepServo() {
        return intakeSmallSweepServo;
    }

    public KServo getIntakeClawServo() {
        return intakeClawServo;
    }

}
