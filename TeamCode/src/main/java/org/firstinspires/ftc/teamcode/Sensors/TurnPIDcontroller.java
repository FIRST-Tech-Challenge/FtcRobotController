package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TurnPIDcontroller {
    private  double targetAngle;
    private double kP, kI, kD;
    private ElapsedTime timer = new ElapsedTime();

    private double lastError = 0;
    private double accumulatedError = 0;

    private double lastTime = -1;
    private double lastSlope = 0;

    public TurnPIDcontroller(double target, double p, double i, double d){
        targetAngle = target;
        kP = p;
        kI = i;
        kD = d;
    }

    public double update(double currentAngle){
        //p
        double error = targetAngle - currentAngle;
        error %=360;
        error += 360;
        error %= 360;
        if(error>180){
            error -= 360;
        }
        // I
        accumulatedError *= Math.signum(error);
        accumulatedError +=error;
        if(Math.abs(error) < 2){
            accumulatedError = 0;
        }
        //D
        double slope = 0;
        if(lastTime >0){
            slope = (error-lastError) / (timer.milliseconds() - lastTime);
        }
        lastSlope = slope;
        lastError = error;
        lastTime = timer.milliseconds();

        double motorPower = 0.1 * Math.signum(error)
                + 0.9 * Math.tanh(kP*error + kI*accumulatedError - kD*slope);
        return motorPower;
    }

    public double getLastSlope() {
        return lastSlope;
    }
}

