package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class actuatorUtils {

    private static DcMotor lift = null; //declare arm

    private static DcMotor arm = null; //declare gripper
    private static CRServo intake = null; //declare dump
    private static Servo wrist = null;
    //test
    public static int upEncode = 1000; //4200 for higher, 2175 for lower-- Max so arm won't overextend and position 3
    private static int restEncode = 2000; //4200 for higher, 2175 for lower-- Max so arm won't overextend and position 3
    public static int downEncode = 0; //4200 for higher, 2175 for lower-- Max so arm won't overextend and position 3
    private static double armPower = 1.0; //Set power to .7 so arm does not go up too fast
    private static int maxEncode = 3100; //4200 for higher, 2175 for lower-- Max so arm won't overextend and position 3
    private static int minEncode = 0; //Minimum so string on arm lift doesn't break and position 0
    private static int lowEncode = 1600; //Minimum so string on arm lift doesn't break and position 0
    private static int highEncode = maxEncode; //Minimum so string on arm lift doesn't break and position 0
    private static double liftPower = .7f; //Set power to .7 so arm does not go up too fast
    private static int parkEncode = 1100;
    enum LiftLevel
    {
        ZERO,
        PARK,
        LOW_BASKET,
        HIGH_BASKET

    }
    enum IntakeModes
    {
        OFF,
        IN,
        OUT

    }
    enum WristModes
    {
        UP,
        DOWN

    }
    enum ArmModes
    {
        UP,
        DOWN,
        REST

    }
    //Initialize actuators
    public static void initializeActuator(DcMotor lift, DcMotor arm,  CRServo intake, Servo wrist) {
        actuatorUtils.lift = lift;
        actuatorUtils.arm = arm;
        actuatorUtils.intake = intake;
        actuatorUtils.wrist = wrist;
    }


    //Method used to close gripper
    public static void setIntake(IntakeModes mode)  {
        if (mode == IntakeModes.IN) {
            intake.setPower(-1.0);
        } else if (mode == IntakeModes.OUT) {
           intake.setPower(1.0);
        } else {
            intake.setPower(0.0);
        }
    }
    public static void setArm(ArmModes mode)   {
        if (mode == ArmModes.UP) {
            arm.setTargetPosition(upEncode); //Lifts arm up so we can move w/o drag
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);


        } else if (mode == ArmModes.REST) {
            arm.setTargetPosition(restEncode); //Lifts arm up so we can move w/o drag
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);


        } else {
            arm.setTargetPosition(downEncode); //Lifts arm up so we can move w/o drag
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);


        }
    }
    public static void setWrist(WristModes mode)  {
        if (mode == WristModes.DOWN) {
            wrist.setPosition(0.333);

        } else {
            wrist.setPosition(0.667);
        }
    }
    public static void setLift(LiftLevel mode) {
        if (mode == LiftLevel.ZERO) {
            lift.setTargetPosition(minEncode); //Lifts arm up so we can move w/o drag
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(liftPower);
        }  else if (mode == LiftLevel.PARK) {
            lift.setTargetPosition(parkEncode); //Lifts arm up so we can move w/o drag
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(liftPower);
        }  else if (mode == LiftLevel.LOW_BASKET) {
            lift.setTargetPosition(lowEncode); //Lifts arm up so we can move w/o drag
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(liftPower);
        } else {
            lift.setTargetPosition(highEncode); //Lifts arm up so we can move w/o drag
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(liftPower);
        }
    }


}
