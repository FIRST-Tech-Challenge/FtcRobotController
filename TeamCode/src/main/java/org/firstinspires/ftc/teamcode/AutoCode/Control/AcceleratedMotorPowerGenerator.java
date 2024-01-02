package org.firstinspires.ftc.teamcode.AutoCode.Control;


public class AcceleratedMotorPowerGenerator
{
    private double accelerationRate;
    private double decelerationRate;
    private double minPow;
    private double targetPower;
    private double currentPower;

    public AcceleratedMotorPowerGenerator(double accelerationRate, double minPow)
    {
        this(accelerationRate, accelerationRate, minPow);
    }

    public AcceleratedMotorPowerGenerator(double accelerationRate, double decelerationRate, double minPow)
    {
        this.accelerationRate = accelerationRate;
        this.decelerationRate = decelerationRate;
        this.minPow = minPow;
    }

    public void setAccelerationRate(double accelerationRate)
    {
        this.accelerationRate = accelerationRate;
    }

    public void stopNow()
    {
        targetPower = 0;
        currentPower = 0;
    }

    public void setTargetPower(double targetPower)
    {
        this.targetPower = targetPower;
    }

    public void update()
    {
        if(currentPower > 0)
        {
            /*
             * Accelerating
             */
            if(currentPower < targetPower)
            {
                currentPower += accelerationRate;
                currentPower = Math.min(currentPower, targetPower);
            }

            /*
             * Decelerating
             */
            else if(currentPower > targetPower)
            {
                currentPower -= decelerationRate;
                currentPower = Math.max(currentPower, targetPower);//2nd was 0
            }
        }
        else if (currentPower < 0)
        {
            /*
             * Decelerating
             */
            if(currentPower < targetPower)
            {
                currentPower += decelerationRate;
                currentPower = Math.min(currentPower, targetPower);//2nd was 0
            }

            /*
             * Accelerating
             */
            else if(currentPower > targetPower)
            {
                currentPower -= accelerationRate;
                currentPower = Math.max(currentPower, targetPower);
            }
        }
        else
        {
            if(targetPower > 0.0)
            {
                currentPower += Math.max(minPow, accelerationRate);
            }
            else if(targetPower < 0.0)
            {
                currentPower -= Math.max(minPow, accelerationRate);
            }
        }
    }

    public double getAccelerationControlledPower()
    {
        return currentPower;
    }
}
