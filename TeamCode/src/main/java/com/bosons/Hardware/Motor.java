package com.bosons.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Motor //DCMotor Wrapper Class with added functionality and usb communication spam avoidance and speedometer
{

    private DcMotor motor;
    private double lastPower;

    public Motor(String name, OpMode op)
    {
        motor = op.hardwareMap.get(DcMotor.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setConstants(DcMotor.RunMode mode, DcMotor.ZeroPowerBehavior zeroPowerBehavior, DcMotor.Direction direction)
    {
        motor.setMode(mode);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        motor.setDirection(direction);
    }



    public void setRunMode(DcMotor.RunMode mode)
    {
        motor.setMode(mode);
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior)
    {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }
    public void setDirection(DcMotor.Direction direction)
    {
        motor.setDirection(direction);
    }
    public void setTargetPosition(int position)
    {
        motor.setTargetPosition(position);
    }

    public double getPower(){
        return lastPower;
    }

    public void setPower(double power)
    {
        if(power == lastPower)
            return;
        if(power < -1)
            power = -1;
        else if(power > 1)
            power = 1;
        motor.setPower(power);
        lastPower = power;
    }

    public void resetEncoder()
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public int getCurrentPosition(){return motor.getCurrentPosition();}

    public int getTargetPosition(){return motor.getTargetPosition();}
    public boolean burnCheck(int acceptableError) {
        int targetDist = Math.abs(getTargetPosition() - getCurrentPosition());
        if (targetDist <= acceptableError) {
            setPower(0);
            return true;
        }
        else {
            return false;
        }
    }
    }