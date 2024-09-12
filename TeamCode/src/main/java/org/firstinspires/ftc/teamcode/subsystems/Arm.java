package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    //Adjustable Constants
    private int ARM_SPEED = 200; //Ticks
    private double POWER = 1; //Motor power ranging from [-1, 1], though realistically [0, 1]

    //Internal variables
    private DcMotor arm;
    private int targetPosition;


    public Arm(HardwareMap hw){
        this(hw, "arm");
    }
    public Arm(HardwareMap hw, String name){
        arm = hw.get(DcMotor.class, name);
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
