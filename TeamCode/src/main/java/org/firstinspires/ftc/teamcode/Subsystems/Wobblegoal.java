package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

public class Wobblegoal {
    //Define Hardware Objects
    public DcMotor WobbleLift=null;
    //public Servo WobbleExtend=null;
    public DcMotor WobbleExtend = null;
    public Servo WobbleGrip=null;

    //Constants
    private static final double     LIFTSPEED   =   0.5;
    private static final int        LIFTUP      =   50; //Number is in Ticks
    private static final int        LIFTDOWN    =   0;
    private static final double     GRIPPEROPEN =   0.6;
    private static final double     GRIPPERCLOSE=   0.2;
    private static final int        ARMEXTEND   =   45; //ticks
    private static final int        ARMCONTRACT =   0; // ticks
    private static final int        ARMCARRY    =   60;
    private static final double     EXTENDSPEED =   .5;

    public void init(HardwareMap hwMap)  {
        WobbleLift=hwMap.get(DcMotor.class,"LiftWobble");
        WobbleExtend=hwMap.get(DcMotor.class,"ArmExtend");
        WobbleGrip=hwMap.get(Servo.class,"Gripper");
        //Positive=up and Negative=down
        WobbleLift.setDirection(DcMotor.Direction.REVERSE);
        WobbleExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        WobbleLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WobbleExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        WobbleLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WobbleExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        WobbleGrip.setPosition(GRIPPEROPEN);

    }

    //// Single operation methods - see below for methods to be called in Opmodes
    public void LiftRise() {
        WobbleLift.setTargetPosition(LIFTUP);
        WobbleLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        WobbleLift.setPower(LIFTSPEED);
    }
    public void LiftLower() {
        WobbleLift.setTargetPosition(LIFTDOWN);
        WobbleLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        WobbleLift.setPower(LIFTSPEED);
    }
    public void GripperOpen()  {

        WobbleGrip.setPosition(GRIPPEROPEN);
    }
    public void GripperClose() {

        WobbleGrip.setPosition(GRIPPERCLOSE);
    }
    public void ArmExtend() {

        WobbleExtend.setTargetPosition(ARMEXTEND);
        WobbleExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        WobbleExtend.setPower(EXTENDSPEED);

    }
    public void ArmContract() {

        WobbleExtend.setTargetPosition(ARMCONTRACT);
        WobbleExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        WobbleExtend.setPower(EXTENDSPEED);
    }
    public void ArmCarryWobble() {

        WobbleExtend.setTargetPosition(ARMCARRY);
        WobbleExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        WobbleExtend.setPower(EXTENDSPEED);
    }

    ///// Multi Function methods to be called by the Opmodes

    public void resetWobble() {
        GripperClose();
        ArmContract();
        LiftLower();
    }

    public void readyToGrabGoal() {
        LiftRise();
        ArmExtend();
        GripperOpen();
        LiftLower();
    }

    public void grabAndLift() {
        GripperClose();
        LiftRise();
    }

    public void lowerAndRelease() {
        LiftLower();
        GripperOpen();
    }
}

