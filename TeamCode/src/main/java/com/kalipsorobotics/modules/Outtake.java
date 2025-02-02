package com.kalipsorobotics.modules;

import android.util.Log;

import com.kalipsorobotics.math.CalculateTickPer;
import com.kalipsorobotics.utilities.KServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {

    private static Outtake single_instance = null;

    //TODO MAKE PRIVATE
    private OpModeUtilities opModeUtilities;
    public DcMotor linearSlide1, linearSlide2;
    public KServo outtakePivotServo;
    public KServo outtakeClawServo;
    public KServo outtakePigeonServo;
    public KServo hangHook1;
    public KServo hangHook2;

    public static final double HOOK1_HANG_POS = 0.8;
    public static final double HOOK2_HANG_POS = 0.18;

    public static final double HOOK1_HANG_READY_POS = 0.7;
    public static final double HOOK2_HANG_READY_POS = 0.28;

    public static final double HOOK1_DOWN_POS = 0.5;
    public static final double HOOK2_DOWN_POS = 0.5;

    public static final double OUTTAKE_PIVOT_TOUCH_BAR_POS = 0.88;
    public static final double OUTTAKE_PIVOT_BASKET_POS = 0.93;
    public static final double OUTTAKE_PIVOT_HALFWAY_BASKET_POS = 0.65;
    public static final double OUTTAKE_PIVOT_PARKING_READY_POS = 0.8;
    public static final double OUTTAKE_PIVOT_DOWN_POS = 0.16;

    public static final double OUTTAKE_PIVOT_TRANSFER_READY_POS = 0.4; //0.4         //increase to go pivot up,
    // decrease
    // to pivot down

    public static final double OUTTAKE_PIVOT_SPECIMEN_HANG_READY_POS = 1;

    public static final double OUTTAKE_PIVOT_WALL_READY_POS = 0.98;
    //decrease to go towards robot, increase to do away from robot
//    public static final double OUTTAKE_PIVOT_SPECIMAN_HANG_POS = 0.8;   //decrease to go towards robot, increase to do away from robot

    public static final double LS_SPECIMEN_HANG_READY_MM = 380; //391 // 406
    public static final double LS_SPECIMEN_PARK_MM = 156;
    public static final double LS_DOWN_POS = -3;
    public static final double LS_SPECIMEN_HANG_DONE_MM = 30;
    public static final double LS_SPECIMEN_CLIP_POS = 246;
    public static final double LS_SAMPLE_BASKET_READY_POS = 675 + 40 + 16 + 75;

    public static final double OUTTAKE_CLAW_CLOSE = 0.616; // 0.996
    public static final double OUTTAKE_CLAW_OPEN = 0.4; //0.78     //increase to make claw close more, decrease to open more


    private Outtake(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        Log.d("Outtake_LS", "init Outtake");

        resetHardwareMap(opModeUtilities.getHardwareMap(), this);

        resetEncoders();
    }

    public static synchronized Outtake getInstance(OpModeUtilities opModeUtilities) {
        if (single_instance == null) {
            single_instance = new Outtake(opModeUtilities);
        } else {
            resetHardwareMap(opModeUtilities.getHardwareMap(), single_instance);
        }
        return single_instance;
    }

    public static void setInstanceNull() {
        single_instance = null;
    }

    private static void resetHardwareMap(HardwareMap hardwareMap, Outtake outtake) {
        outtake.linearSlide1 = hardwareMap.dcMotor.get("linearSlide1");
        outtake.linearSlide2 = hardwareMap.dcMotor.get("linearSlide2");
        outtake.outtakePivotServo = new KServo(hardwareMap.servo.get("outtakePivot"), 60/0.11, 255,
                0, false);
        outtake.outtakeClawServo = new KServo(hardwareMap.servo.get("outtakeClaw"), 60/0.11, 255, //mini axon
                0, false);
//        outtakePigeonServo = new KServo(opModeUtilities.getHardwareMap().servo.get("outtakePigeonServo"), 60/0.25, 300,
//                0, false);
        outtake.hangHook1 = new KServo(hardwareMap.servo.get("hang1"), 60/0.25, 300,
                0, false);
        outtake.hangHook2 = new KServo(hardwareMap.servo.get("hang2"), 60/0.25, 300,
                0, false);

        outtake.linearSlide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.linearSlide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtake.linearSlide1.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake.linearSlide2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void resetEncoders() {
        linearSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void init() {
        getOuttakeClaw().setPosition(OUTTAKE_CLAW_CLOSE);
        getOuttakePivotServo().setPosition(OUTTAKE_PIVOT_TRANSFER_READY_POS);
        getHangHook1().setPosition(HOOK1_DOWN_POS);
        getHangHook2().setPosition(HOOK2_DOWN_POS);
    }


    public double getCurrentPosMm() {
        return CalculateTickPer.ticksToMmLS(getLinearSlide1().getCurrentPosition());
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
