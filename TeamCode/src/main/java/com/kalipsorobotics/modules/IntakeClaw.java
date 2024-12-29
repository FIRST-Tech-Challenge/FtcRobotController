package com.kalipsorobotics.modules;

import com.kalipsorobotics.utilities.KServo;
import com.kalipsorobotics.utilities.OpModeUtilities;

public class IntakeClaw {
    public static final double INTAKE_BIG_SWEEP_PARALLEL_TO_ROBOT = 0.5;

    public static final double INTAKE_BIG_PIVOT_INTAKE_READY_POS = 0.67;       //increase to go down, decrease to go up

    public static final double INTAKE_SMALL_PIVOT_INTAKE_READY_POS = 0.77;

    public static final double INTAKE_SMALL_SWEEP_INTAKE_READY_POS = 0.8; //decrease to move forward, increase to move back


    public static final double INTAKE_LINKAGE_EXTEND_POS = 0.67;     //increase to retract more


    public static final double INTAKE_BIG_SWEEP_TRANSFER_READY_POS = 0.5;

    public static final double INTAKE_BIG_PIVOT_TRANSFER_READY_POS = 0.55;       //increase to go down, decrease to go up

    public static final double INTAKE_SMALL_PIVOT_TRANSFER_READY_POS = 0.1;     //decrease to pivot back of the robot, increase to pivot to front

    public static final double INTAKE_SMALL_SWEEP_TRANSFER_READY_POS = 0.77;

    public static final double INTAKE_BIG_PIVOT_GRAB_SAMPLE_POS = 0.88;     //increase to go down, decrease to go up

    public static final double INTAKE_SMALL_PIVOT_GRAB_SAMPLE_POS = 0.71;   //decrease to move forward, increase to move back

    public static final double INTAKE_BIG_PIVOT_GRAB_SAMPLE_3_POS = 0.455;

    public static final double INTAKE_SMALL_PIVOT_GRAB_SAMPLE_3_POS = 0.61;


    private final OpModeUtilities opModeUtilities;

    private KServo intakeLinkageServo;

    private KServo intakeBigSweepServo;

    private KServo intakeBigPivotServo;
    private KServo intakeSmallPivotServo;

    private KServo intakeSmallSweepServo;

    private KServo intakeClawServo;


    public static final double INTAKE_CLAW_CLOSE = 0.36;
    public static final double INTAKE_CLAW_OPEN = 0.06; //increase to close claw more

    public static final double INTAKE_SMALL_SWEEP_RETRACT_POS = 0.77;
    public static final double INTAKE_SMALL_SWEEP_THIRD_SAMPLE_BASKET_GRAB_POS =0.645;

    public static final double INTAKE_BIG_PIVOT_RETRACT_POS = 0;
    public static final double INTAKE_SMALL_PIVOT_RETRACT_POS = 0.78;

    public static final double INTAKE_LINKAGE_IN_POS = 0.95;
    public static final double INTAKE_LINKAGE_OUT_POS = 0.57;




    public IntakeClaw(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        setUpHardware();
    }

    private void setUpHardware() {

        intakeLinkageServo = new KServo(opModeUtilities.getHardwareMap().servo.get("intakeLinkage"), 45/1,
                130,
                0, false);

        intakeBigSweepServo = new KServo(opModeUtilities.getHardwareMap().servo.get("intakeBigSweep"), 60/0.25,
                300,
                0, false);

        intakeBigPivotServo = new KServo(opModeUtilities.getHardwareMap().servo.get("intakeBigPivot"), 60/0.11,
                130,
                0, false);

        intakeSmallPivotServo = new KServo(opModeUtilities.getHardwareMap().servo.get("intakeSmallPivot"), 60/0.11,
                255,
                0, false);

        intakeSmallSweepServo = new KServo(opModeUtilities.getHardwareMap().servo.get("intakeSmallSweep"), 60/0.25,
                300,
                0, false);

        intakeClawServo = new KServo(opModeUtilities.getHardwareMap().servo.get("intakeClaw"), 60/0.11,     //mini axon
                255,
                0, false);
    }

    public void init() {
        getIntakeLinkageServo().setPosition(IntakeClaw.INTAKE_LINKAGE_IN_POS);
        getIntakeBigSweepServo().setPosition(0.5);
        getIntakeBigPivotServo().setPosition(IntakeClaw.INTAKE_BIG_PIVOT_RETRACT_POS);
        getIntakeSmallPivotServo().setPosition(IntakeClaw.INTAKE_SMALL_PIVOT_RETRACT_POS);
        getIntakeSmallSweepServo().setPosition(IntakeClaw.INTAKE_SMALL_SWEEP_RETRACT_POS);
        getIntakeClawServo().setPosition(IntakeClaw.INTAKE_CLAW_OPEN);
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
