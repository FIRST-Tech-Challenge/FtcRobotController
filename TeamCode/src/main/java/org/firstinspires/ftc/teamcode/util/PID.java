package org.firstinspires.ftc.teamcode.util;

/**
 * This is the PID class to use for the robot.
 */
public class PID
{
    private long timeStamp, prevTimeStamp;
    private double kP, kI, kD, kF;
    private double error, previousError;
    private final double deadband = .05;
    private double min = -1, max = 1;
    private double setpoint = 0;

    //https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html#using-feedforward-to-control-mechanisms
    //https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#arbitrary-feed-forward
    
    public PID(double kP, double kI, double kD, double kF)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        previousError = 0;
    }
    
    public PID(double kP, double kI, double kD, double kF, double max, double min)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.max = max;
        this.min = min;
        previousError = 0;
    }

    /**
     * Calculates the next output for the pid, then returns the output.
     *
     * @return
     */
    public double update(double input)
    {
        error = setpoint - input;
        double output = calculateP();// - calculateD(); // + calculateI
        previousError = error;
        return Util.trim(output, min, max);
    }
    
    public double update(double input, double arbFF)
    {
        error = setpoint - input;
        double output = calculateP() + arbFF;// - calculateD(); // + calculateI
        previousError = error;
        return Util.trim(output, min, max);
    }

    /**
     * @return the error multiplied by the kP term.
     */
    public double calculateP()
    {
        return kP * error;
    }

    /**
     * The rate of change for the error with respect to time.
     *
     * @return
     */
    public double calculateD()
    {
        // TODO if first time call, reset timer
        return kD * ((error - previousError));// / (double) (timeStamp - prevTimeStamp));
    }
    
    double max_i = 0.1; //?
    private double calculateI()
    {
        // TODO if first time call, reset timer
        double ci = kI * (error * ((double) (timeStamp - prevTimeStamp)));
        if( ci > max_i)
            ci = max_i;
        else if(ci < -max_i)
            ci = -max_i;
        return ci;
    }
    
    //Getters and setters for variables.
    public double getP()
    {
        return kP;
    }

    public double getI()
    {
        return kI;
    }

    public double getD()
    {
        return kD;
    }

    public double getF()
    {
        return kF;
    }

    public double getMax()
    {
        return max;
    }

    public double getMin()
    {
        return min;
    }

    public void setP(double p)
    {
        kP = p;
    }

    public void setI(double i)
    {
        kI = i;
    }

    public void setD(double d)
    {
        kD = d;
    }

    public void setF(double f)
    {
        kF = f;
    }

    public void setMax(double max)
    {
        this.max = max;
    }

    public void setMin(double min)
    {
        this.min = min;
    }

    public void setSetpoint(double target)
    {
        setpoint = target;
    }
}
