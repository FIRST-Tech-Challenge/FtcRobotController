package com.kalipsorobotics.modules;

import android.util.Log;

import com.kalipsorobotics.utilities.KServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.kalipsorobotics.utilities.OpModeUtilities;

public class Outtake {

    private static Outtake single_instance = null;

    //TODO MAKE PRIVATE
    private final OpModeUtilities opModeUtilities;
    public DcMotor linearSlide1, linearSlide2;
    public KServo outtakePivotServo;
    public KServo outtakeClawServo;
    public KServo outtakePigeonServo;
    public KServo hangHook1;
    public KServo hangHook2;

    public static final double HOOK1_HANG_POS = 0.105;
    public static final double HOOK2_HANG_POS = 0.79;

    public static final double HOOK1_DOWN_POS = 0.555;
    public static final double HOOK2_DOWN_POS = 0.35;

    public static final double OUTTAKE_PIVOT_TOUCH_BAR_POS = 0.85;
    public static final double OUTTAKE_PIVOT_BASKET_POS = 0.93;
    public static final double OUTTAKE_PIVOT_DOWN_POS = 0.16;

    public static final double OUTTAKE_PIVOT_TRANSFER_READY_POS = 0.38;         //increase to go pivot up, decrease to pivot down

    public static final double OUTTAKE_PIVOT_SPECIMEN_HANG_READY_POS = 1;

    public static final double OUTTAKE_PIVOT_WALL_READY_POS = 0.97;
    //decrease to go towards robot, increase to do away from robot
//    public static final double OUTTAKE_PIVOT_SPECIMAN_HANG_POS = 0.8;   //decrease to go towards robot, increase to do away from robot

    public static final double LS_SPECIMAN_HANG_READY_MM = 380;
    public static final double LS_DOWN_POS = 0;
    public static final double LS_SPECIMEN_HANG_DONE_MM = 30;
    public static final double LS_SPECIMEN_CLIP_POS = 230;
    public static final double LS_SAMPLE_BASKET_READY_POS = 675;

    public static final double OUTTAKE_CLAW_CLOSE = 0.84;
    public static final double OUTTAKE_CLAW_OPEN = 0.65;     //increase to make claw close more, decrease to open more



    private Outtake(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        Log.d("Outtake_LS", "init Outtake");
        setUpHardware();
    }

    public static synchronized Outtake getInstance(OpModeUtilities opModeUtilities) {
        if (single_instance == null) {
            single_instance = new Outtake(opModeUtilities);
        }
        return single_instance;
    }

    private void setUpHardware() {
        linearSlide1 = opModeUtilities.getHardwareMap().dcMotor.get("linearSlide1");
        linearSlide2 = opModeUtilities.getHardwareMap().dcMotor.get("linearSlide2");
        outtakePivotServo = new KServo(opModeUtilities.getHardwareMap().servo.get("outtakePivot"), 60/0.11, 255,
                0, false);
        outtakeClawServo = new KServo(opModeUtilities.getHardwareMap().servo.get("outtakeClaw"), 60/0.11, 255, //mini axon
                0, false);
//        outtakePigeonServo = new KServo(opModeUtilities.getHardwareMap().servo.get("outtakePigeonServo"), 60/0.25, 300,
//                0, false);
        hangHook1 = new KServo(opModeUtilities.getHardwareMap().servo.get("hang1"), 60/0.25, 300,
                0, false);
        hangHook2 = new KServo(opModeUtilities.getHardwareMap().servo.get("hang2"), 60/0.25, 300,
                0, false);

        linearSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linearSlide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearSlide1.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlide2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void init() {
        getOuttakeClaw().setPosition(OUTTAKE_CLAW_CLOSE);
        getOuttakePivotServo().setPosition(OUTTAKE_PIVOT_TRANSFER_READY_POS);
    }

    public DcMotor getLinearSlide1() {
        return linearSlide1;
    }
    public DcMotor getLinearSlide2() {
        return linearSlide2;
    }
    public KServo getOuttakePivotServo() {
        return outtakePivotServo;
    }
    public KServo getOuttakeClaw() {
        return outtakeClawServo;
    }
    public KServo getOuttakePigeonServo() {
        return outtakePigeonServo;
    }
    public KServo getHangHook1() {return hangHook1;}
    public KServo getHangHook2() {return hangHook2;}

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }
}
