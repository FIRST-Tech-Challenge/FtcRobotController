package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    //Adjust able Constants
    public int LIFT_SPEED = 20; //ticks per call
    public double POWER = 1;
    public int MEDIUM_HEIGHT = 1000;
    public int HIGH_HEIGHT = 2000;

    //Internal variables
    private DcMotor liftLeft, liftRight;
    private int targetPosition;


    public Lift(HardwareMap hw){
        this(hw, "liftLeft", "liftRight");
    }

    public Lift(HardwareMap hw, String nameLeft, String nameRight){
        this.liftLeft = hw.get(DcMotor.class, nameLeft);
        this.liftRight = hw.get(DcMotor.class, nameRight);
        this.liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        resetLift(liftLeft);
        resetLift(liftRight);
    }

    public int moveLift(double power){
        targetPosition += (power * LIFT_SPEED);
        liftLeft.setTargetPosition(targetPosition);
        liftRight.setTargetPosition(targetPosition);
        return targetPosition;
    }

    public int currentPos(){
        return liftLeft.getCurrentPosition();
    }
    public void goToMedium(){
        liftLeft.setTargetPosition(MEDIUM_HEIGHT);
        liftRight.setTargetPosition(MEDIUM_HEIGHT);
    }

    public void goToHigh(){
        liftLeft.setTargetPosition(HIGH_HEIGHT);
        liftRight.setTargetPosition(HIGH_HEIGHT);
    }

    public int getCurrentPosition(){
        return liftLeft.getCurrentPosition();
    }
    public void resetLift(DcMotor liftMotor){
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetPosition = liftMotor.getCurrentPosition();
        liftMotor.setTargetPosition(targetPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(POWER);
    }
}
