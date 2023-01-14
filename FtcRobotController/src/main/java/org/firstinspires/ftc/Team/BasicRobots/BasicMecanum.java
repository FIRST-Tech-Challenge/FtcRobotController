package org.firstinspires.ftc.Team.BasicRobots;

//This file is the basic framework for a robot that drives on mecanum wheels

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.*;

public class BasicMecanum {

    public DcMotor[] motors = {null, null, null, null};
    public int RFMotor = 0;
    public int RBMotor = 1;
    public int LFMotor = 2;
    public int LBMotor = 3;

    String[] names = {"RFMotor", "RBMotor", "LFMotor", "LBMotor"};

    public double mult = 0.29;

    public void init(HardwareMap Map) {

        for (int i = 0; i < 5; i++) {
            motors[i] = Map.dcMotor.get(names[i]);
            motors[i].setPower(0.0);
            motors[i].setDirection(DcMotor.Direction.FORWARD);
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    //Move straight function

    public void moveStraight(double power) {
        motors[2].setPower(power * mult);
        motors[0].setPower(power);
        motors[3].setPower(power * mult);
        motors[1].setPower(power);
    }

    //Move left function

    public void moveLeft(double power) {
        motors[2].setPower(-power * mult);
        motors[0].setPower(power);
        motors[3].setPower(power * mult);
        motors[1].setPower(-power);
    }

    //Move right function

    public void moveRight(double power) {
        motors[2].setPower(power * mult);
        motors[0].setPower(-power);
        motors[3].setPower(-power * mult);
        motors[1].setPower(power);
    }

    //Move back function

    public void moveBackwards(double power) {
        motors[2].setPower(-power * mult);
        motors[0].setPower(-power);
        motors[3].setPower(-power * mult);
        motors[1].setPower(-power);
    }

    //Stop moving function

    public void stop() {
        for (DcMotor motor: motors) {
            motor.setPower(0);
        }
    }

    //Pivot in place based on input from two buttons (left/right (booleans)) and a power setting (0-1 (double-precision float))

    public void pivotTurn(double power, boolean rightBumper, boolean leftBumper) {
        power = power*0.5;
        if(rightBumper && leftBumper) {
            this.stop();
        } else if(leftBumper) {
            motors[0].setPower(power);
            motors[2].setPower(-power * mult);
            motors[1].setPower(power);
            motors[3].setPower(-power * mult);
        } else if(rightBumper) {
            motors[0].setPower(-power);
            motors[2].setPower(power * mult);
            motors[1].setPower(-power);
            motors[3].setPower(power * mult);
        }
    }

    //Strafe in all 8 directions based on button inputs (up/down/left/right (boolean)) and a power setting (0-1 (double-precision float))

    public void octoStrafe(double power, boolean up, boolean down, boolean left, boolean right) {
        if (up) {
            if (right) {
                motors[0].setPower(power);
                motors[2].setPower(0);
                motors[1].setPower(0);
                motors[3].setPower(power * mult);
            } else if (left) {
                motors[0].setPower(0);
                motors[2].setPower(power * mult);
                motors[1].setPower(power);
                motors[3].setPower(0);
            } else {
                motors[0].setPower(power);
                motors[2].setPower(power * mult);
                motors[1].setPower(power);
                motors[3].setPower(power * mult);
            }
        } else if (down) {
            if (right) {
                motors[0].setPower(0);
                motors[2].setPower(-power * mult);
                motors[1].setPower(-power);
                motors[3].setPower(0);
            } else if (left) {
                motors[0].setPower(-power);
                motors[2].setPower(0);
                motors[1].setPower(0);
                motors[3].setPower(-power * mult);
            } else {
                motors[0].setPower(-1);
                motors[2].setPower(-power * mult);
                motors[1].setPower(-1);
                motors[3].setPower(-power * mult);
            }
        }
        else {
            if (right) {
                motors[0].setPower(-power);
                motors[2].setPower(power * mult);
                motors[1].setPower(power);
                motors[3].setPower(-power * mult);
            } else if (left) {
                motors[0].setPower(power);
                motors[2].setPower(-power * mult);
                motors[1].setPower(-power);
                motors[3].setPower(power * mult);
            }
        }
    }
}