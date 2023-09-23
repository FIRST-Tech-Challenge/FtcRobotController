package org.firstinspires.ftc.teamcode.drivebase;

/**
 * Created by Michael on 10/10/2017.
 */

public class MotorPowers
{
    public double frontRight;
    public double frontLeft;
    public double rearLeft;
    public double rearRight;

    public MotorPowers()
    {

    }

    public MotorPowers (double leftPower, double rightPower)
    {
        frontLeft = leftPower;
        rearLeft = leftPower;
        frontRight = rightPower;
        rearRight = rightPower;
    }

    public MotorPowers (double power)
    {
        frontLeft = power;
        rearLeft = power;
        frontRight = power;
        rearRight = power;
    }

    public MotorPowers (double frontLeft, double frontRight, double rearLeft, double rearRight)
    {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;
    }

    public void add(double left, double right)
    {
        frontLeft += left;
        rearLeft += left;
        frontRight += right;
        rearRight += right;
    }

    public String toString()
    {
        return "FL: " + frontLeft + " FR: " + frontRight + " RL: " + rearLeft + " RR: " + rearRight;
    }
}