package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class actuatorUtils {

    private static DcMotor lift = null; //declare arm

    private static Servo leftArm = null; //declare gripper
    private static Servo rightArm = null; //declare dump
    private static CRServo intake = null; //declare dump
    //test
    private static int maxEncode = 4180; //4200 for higher, 2175 for lower-- Max so arm won't overextend and position 3
    private static int minEncode = 0; //Minimum so string on arm lift doesn't break and position 0
    private static int lowEncode = 1980; //Minimum so string on arm lift doesn't break and position 0
    private static int highEncode = maxEncode; //Minimum so string on arm lift doesn't break and position 0
    private static double liftPower = .7f; //Set power to .7 so arm does not go up too fast

    enum LiftLevel
    {
        ZERO,
        LOW_BASKET,
        HIGH_BASKET

    }
    enum IntakeModes
    {
        OFF,
        IN,
        OUT

    }
    enum ArmModes
    {
        UP,
        DOWN,
        REST

    }
    //Initialize actuators
    public static void initializeActuator(DcMotor lift, Servo leftArm, Servo rightArm, CRServo intake) {
        actuatorUtils.lift = lift;
        actuatorUtils.leftArm = leftArm;
        actuatorUtils.rightArm = rightArm;
        actuatorUtils.intake = intake;

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
            leftArm.setPosition(0.333);
            rightArm.setPosition(0.667);
        } else if (mode == ArmModes.REST) {
            leftArm.setPosition(0.5);
            rightArm.setPosition(0.5);
        } else {
            leftArm.setPosition(0.667);
            rightArm.setPosition(0.333);
        }
    }
    public static void setLift(LiftLevel mode) {
        if (mode == LiftLevel.ZERO) {
            lift.setTargetPosition(minEncode); //Lifts arm up so we can move w/o drag
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
