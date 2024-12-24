package com.kalipsorobotics.modules;

import com.kalipsorobotics.utilities.KServo;
import com.kalipsorobotics.utilities.OpModeUtilities;

public class IntakeClaw {
    public static final double INTAKE_BIG_SWEEP_PARALLEL_TO_ROBOT = 0.5;

    public static final double INTAKE_BIG_PIVOT_PARALLEL_TO_GROUND_POS = 0.7;       //increase to go down, decrease to go up

    public static final double INTAKE_SMALL_PIVOT_PERPENDICULAR_TO_GROUND_POS = 0.77;

    public static final double INTAKE_SMALL_SWEEP_PERPENDICULAR_TO_ROBOT_POS = 0.77;

    public static final double INTAKE_CLAW_OPEN_POS = 0.5;

    public static final double INTAKE_LINKAGE_EXTEND_POS = 0.5;

    private final OpModeUtilities opModeUtilities;


    private KServo intakeLinkageServo;

    private KServo intakeBigSweepServo;

    private KServo intakeBigPivotServo;
    private KServo intakeSmallPivotServo;

    private KServo intakeSmallSweepServo;

    private KServo intakeClawServo;


    public static final double INTAKE_CLAW_CLOSE = 0.365;
    public static final double INTAKE_CLAW_OPEN = 0.05;

    public static final double INTAKE_BIG_PIVOT_INTAKE_READY_POS = 0.777;
    public static final double INTAKE_SMALL_PIVOT_INTAKE_READY_POS = 0.77;
    public static final double INTAKE_SMALL_SWEEP_RETRACT_POS = 0.77;

    public static final double INTAKE_BIG_PIVOT_RETRACT_POS = 0;
    public static final double INTAKE_SMALL_PIVOT_RETRACT_POS = 0.83;

    public static final double INTAKE_BIG_PIVOT_TRANSFER_POS = 0.64;
    public static final double INTAKE_SMALL_PIVOT_TRANSFER_POS = 0.045;

    public static final double INTAKE_LINKAGE_IN_POS = 0.95;
    public static final double INTAKE_LINKAGE_OUT_POS = 0.57;


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
