package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    //Adjust able Constants
    public int LIFT_SPEED = 200; //ticks per call
    public double POWER = 1;
    public int MEDIUM_HEIGHT = 1000;
    public int HIGH_HEIGHT = 2000;

    //Internal variables
    private DcMotor lift;
    private int targetPosition;


    public Lift(HardwareMap hw){
        this(hw, "lift");
    }

    public Lift(HardwareMap hw, String name){
        this.lift = hw.get(DcMotor.class, name);

        resetLift();
    }

    public void moveLift(double power){
        targetPosition += (power * LIFT_SPEED);
        lift.setTargetPosition(targetPosition);
    }

    public void goToMedium(){
        lift.setTargetPosition(MEDIUM_HEIGHT);
    }

    public void goToHigh(){
        lift.setTargetPosition(HIGH_HEIGHT);
    }

    public int getCurrentPosition(){
        return lift.getCurrentPosition();
    }
    public void resetLift(){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetPosition = lift.getCurrentPosition();
        lift.setTargetPosition(targetPosition);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(POWER);
    }
}
