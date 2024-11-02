package com.bosons.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveTrain
{

    private Motor fl;
    private Motor bl;
    private Motor fr;
    private Motor br;

    private double drivePowerCoefficient = 1;//default
    private double turnPowerCoefficient = 1;//default

    //private BNO055IMU IMU;

    public DriveTrain(OpMode op)
    {

        fl = new Motor("fl", op);
        bl = new Motor("bl", op);
        fr = new Motor("fr", op);
        br = new Motor("br", op);



        fl.setConstants(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, DcMotorSimple.Direction.FORWARD);
        bl.setConstants(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, DcMotorSimple.Direction.FORWARD);
        fr.setConstants(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, DcMotorSimple.Direction.REVERSE);
        br.setConstants(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double power, double theta, double turn)
    {//vroom


        double x = power*drivePowerCoefficient * Math.cos(theta) * 1.5;
        double y = power*drivePowerCoefficient * Math.sin(theta);



        double flPower = y + x + turn*turnPowerCoefficient;
        double blPower = y - x + turn*turnPowerCoefficient;
        double frPower = y - x - turn*turnPowerCoefficient;
        double brPower = y + x - turn*turnPowerCoefficient;

        double maxPower = 1;
        if(Math.abs(flPower) > maxPower || Math.abs(blPower) > maxPower
                || Math.abs(frPower) > maxPower || Math.abs(brPower) > maxPower)
        {
            double max = Math.max(Math.abs(flPower), Math.abs(blPower));
            max = Math.max(max, Math.abs(frPower));
            max = Math.max(max, Math.abs(brPower));

            flPower /= max;
            blPower /= max;
            frPower /= max;
            brPower /= max;
        }

        fl.setPower(flPower);
        bl.setPower(blPower);
        fr.setPower(frPower);
        br.setPower(brPower);
    }

    public void setDrivePowerCoefficient(double coefficient){
        drivePowerCoefficient = coefficient;
    }
    public void setTurnPowerCoefficient(double coefficient){
        turnPowerCoefficient = coefficient;
    }

}