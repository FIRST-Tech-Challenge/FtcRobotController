package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
//TODO:Reconfigure Arm Class to utilize servos instead
public class Arm {

    //Adjustable Constants
    private int ARM_SPEED = 200; //Ticks
    private double POWER = 1; //Motor power ranging from [-1, 1], though realistically [0, 1]

    //Internal variables
    private DcMotor armLeft, armRight, arm;
    private int targetPosition;


    public Arm(HardwareMap hw){
        this(hw, "armLeft", "armRight");
    }
    public Arm(HardwareMap hw, String nameLeft, String nameRight){
        armLeft = hw.get(DcMotor.class, nameLeft);
        armRight = hw.get(DcMotor.class, nameRight);
        resetArm();
    }

    public void changeHeight(double power){
        targetPosition += (power * ARM_SPEED);
        arm.setTargetPosition(targetPosition);
    }

    public void resetArm(){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetPosition = arm.getCurrentPosition();
        arm.setTargetPosition(targetPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(POWER);
    }
}
