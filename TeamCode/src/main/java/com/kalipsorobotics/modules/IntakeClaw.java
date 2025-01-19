package com.kalipsorobotics.modules;

import com.kalipsorobotics.utilities.KServo;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeClaw {
    private static IntakeClaw single_instance = null;

    //INTAKE READY

    public static final double INTAKE_BIG_SWEEP_PARALLEL_TO_ROBOT = 0.5;

    public static final double INTAKE_BIG_PIVOT_INTAKE_READY_POS = 0.67;       //increase to go down, decrease to go up

    public static final double INTAKE_SMALL_PIVOT_INTAKE_READY_POS = 0.77;

    public static final double INTAKE_SMALL_SWEEP_INTAKE_READY_POS = 0.46; //decrease to move more horizontal, increase to move more vertical

    //INTAKE SAMPLE IN FUNNEL

    public static final double INTAKE_SMALL_PIVOT_FUNNEL_READY_POS = 0.1;

    public static final double INTAKE_BIG_PIVOT_FUNNEL_READY_POS = 0.075; //decrease to pivot back to the robot more

    public static final double INTAKE_SMALL_PIVOT_FUNNEL_GRAB_POS = 0.14;

    public static final double INTAKE_BIG_PIVOT_FUNNEL_GRAB_POS = 0.01;

    public static final double INTAKE_LINKAGE_EXTEND_POS = 0.62; //increase to retract more

    //TRANSFER READY

    public static final double INTAKE_BIG_SWEEP_TRANSFER_READY_POS = 0.48;

    public static final double INTAKE_BIG_PIVOT_TRANSFER_READY_POS = 0.5;       //increase to go down, decrease to go up

    public static final double INTAKE_SMALL_PIVOT_TRANSFER_READY_POS = 0.19;     //decrease to pivot back of the robot, increase to pivot to front

    public static final double INTAKE_SMALL_SWEEP_TRANSFER_READY_POS = 0.46;

    //INTAKE ACTION

    public  static final double INTAKE_SMALL_SWEEP_VERTICAL_POS = 0.77; //increase to move more horizontal, decrease to move more vertical

    public static final double INTAKE_BIG_PIVOT_GRAB_SAMPLE_POS = 0.9; //0.85    //increase to go down, decrease to go up

    public static final double INTAKE_SMALL_PIVOT_GRAB_SAMPLE_POS = 0.73;   //decrease to move forward, increase to move back

    public static final double INTAKE_BIG_PIVOT_GRAB_SAMPLE_3_POS = 0.455;

    public static final double INTAKE_SMALL_PIVOT_GRAB_SAMPLE_3_POS = 0.61;


    private OpModeUtilities opModeUtilities;

    private KServo intakeLinkageServo;

    private KServo intakeBigSweepServo;

    private KServo intakeBigPivotServo;
    private KServo intakeSmallPivotServo;

    private KServo intakeSmallSweepServo;

    private KServo intakeClawServo;


    public static final double INTAKE_CLAW_CLOSE = 0.35;
    public static final double INTAKE_CLAW_OPEN = 0.07; //increase to close claw more

    public static final double INTAKE_SMALL_SWEEP_SPECIMEN_POS = 0.35;
    public static final double INTAKE_SMALL_SWEEP_RETRACT_POS = 0.43; // Horizontal from Crossbar (parallel)
    public static final double INTAKE_SMALL_SWEEP_THIRD_SAMPLE_BASKET_GRAB_POS =0.645;

    public static final double INTAKE_BIG_PIVOT_RETRACT_POS = 0;
    public static final double INTAKE_SMALL_PIVOT_RETRACT_POS = 0.73;

    public static final double INTAKE_LINKAGE_IN_POS = 0.96; //0.95
    public static final double INTAKE_LINKAGE_MID_POS = 0.87;
    public static final double INTAKE_LINKAGE_OUT_POS = 0.57;




    private IntakeClaw(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;

        resetHardwareMap(opModeUtilities.getHardwareMap(), this);
    }

    public static synchronized IntakeClaw getInstance(OpModeUtilities opModeUtilities) {
        if (single_instance == null) {
            single_instance = new IntakeClaw(opModeUtilities);
        } else {
            resetHardwareMap(opModeUtilities.getHardwareMap(), single_instance);
        }
        return single_instance;
    }

    public static void setInstanceNull() {
        single_instance = null;
    }

    private static void resetHardwareMap(HardwareMap hardwareMap, IntakeClaw intakeClaw) {
        intakeClaw.intakeLinkageServo = new KServo(hardwareMap.servo.get("intakeLinkage"), 45/1,
                130,
                0, false);

        intakeClaw.intakeBigSweepServo = new KServo(hardwareMap.servo.get("intakeBigSweep"), 60/0.25,
                300,
                0, false);

        intakeClaw.intakeBigPivotServo = new KServo(hardwareMap.servo.get("intakeBigPivot"), 60/0.11,
                130,
                0, false);

        intakeClaw.intakeSmallPivotServo = new KServo(hardwareMap.servo.get("intakeSmallPivot"), 60/0.11,
                255,
                0, false);

        intakeClaw.intakeSmallSweepServo = new KServo(hardwareMap.servo.get("intakeSmallSweep"), 60/0.25,
                300,
                0, false);

        intakeClaw.intakeClawServo = new KServo(hardwareMap.servo.get("intakeClaw"), 60/0.25,     //mini axon
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
