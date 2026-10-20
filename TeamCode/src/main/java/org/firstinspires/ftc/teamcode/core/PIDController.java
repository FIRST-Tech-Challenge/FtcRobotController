package org.firstinspires.ftc.teamcode.core;


public class PIDController {

    private double kP;
    private double maxError;

    private double kI;
    private double errorSum = 0;
    private double kIActiveZone = 0;

    private double kD;
    private double lastError;

    private boolean canLoop = false;
    private double loopMin;
    private double loopMax;


    public PIDController(double kP, double kI, double kD)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        maxError = -1;
        errorSum = 0;
        kIActiveZone = -1;
    }
    public PIDController(double kP, double maxError, double kI, double kD)
    {
        this.kP = kP;
        this.maxError = maxError;
        this.kI = kI;
        this.kD = kD;

        errorSum = 0;
        kIActiveZone = -1;
    }
    public PIDController(double kP, double maxError, double kI, double kIActiveZone, double kD)
    {
        this.kP = kP;
        this.maxError = maxError;
        this.kI = kI;
        this.kIActiveZone = kIActiveZone;
        this.kD = kD;

        errorSum = 0;
    }

    public double Calculate(double currentPosition, double targetPosition, double deltaTime)
    {


        double error = targetPosition - currentPosition;
        double errorRate = 0;

        // Proportional
        if (kP != 0)
        {
            if (maxError >= 0 && error > maxError)
            {
                error = maxError;
            }
        }

        // Interval
        if (kI != 0)
        {
            if (kIActiveZone >= 0)
            {
                if (Math.abs(error) < kIActiveZone)
                {
                    errorSum += error * deltaTime;
                }
                else
                {
                    errorSum = 0;
                }
            }
            else
            {
                errorSum += error * deltaTime;
            }
        }

        // Derivative
        if (kD != 0)
        {
            if (error < maxError)
            {
                errorRate = (error - lastError) / deltaTime;
                lastError = errorRate;
            }
        }

        return (kP * error) + (kI * errorSum) + (kD * errorRate);
    }

    public void Reset()
    {
        errorSum = 0;
    }

    public void SetLoop(boolean canLoop, double loopMin, double loopMax)
    {
        this.canLoop = canLoop;
        this.loopMin = loopMin;
        this.loopMax = loopMax;
    }

}