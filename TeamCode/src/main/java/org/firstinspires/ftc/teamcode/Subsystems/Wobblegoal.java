package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wobblegoal {
    //Define Hardware Objects
    public DcMotor WobbleLift=null;
    public Servo WobbleExtend=null;
    public Servo WobbleGrip=null;

    //Constants
private static final double LIFTSPEED=0.5;
private static final int LIFTUP=50; //Number is in Ticks
private static final int LIFTDOWN=0;
private static final double GRIPPEROPEN=0.2;
private static final double GRIPPERCLOSE=0.7;
private static final double ARMEXTEND=0.3;
private static final double ARMCONTRACT=0.8;

    public void init(HardwareMap hwMap)  {
        WobbleLift=hwMap.get(DcMotor.class,"LiftWobble");
        WobbleExtend=hwMap.get(Servo.class,"ArmExtend");
        WobbleGrip=hwMap.get(Servo.class,"Gripper");
        //Positive=up and Negative=down
        WobbleLift.setDirection(DcMotor.Direction.REVERSE);
        WobbleLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        WobbleLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
}
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
    public void GripperOpen() {
        WobbleGrip.setPosition(GRIPPEROPEN);
    }
    public void GripperClose() {
        WobbleGrip.setPosition(GRIPPERCLOSE);
    }
    public void ArmExtend() {
        WobbleExtend.setPosition(ARMEXTEND);
    }
    public void ArmContract() {
        WobbleExtend.setPosition(ARMCONTRACT);
    }
    }

