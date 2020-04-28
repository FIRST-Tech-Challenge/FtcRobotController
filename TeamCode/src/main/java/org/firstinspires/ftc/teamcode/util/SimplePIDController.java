package org.firstinspires.ftc.teamcode.util;

public class SimplePIDController {

    public double kP, kI, kD;

    private double err_sum = 0, last_error = 0, last_time = 0;

    public SimplePIDController()
    {
        kP = kI = kD = 0;
    }

    public SimplePIDController( double kP, double kI, double kD)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double update(double error)
    {
        return update(error, System.nanoTime() / Math.pow(10, 9));
    }

    public double update(double error, double time)
    {
        double control_update = 0;

        // P Term
        double p_term = kP * error;

        // I Term
        double dt = (time - last_time);
        err_sum += ((error + last_error) / 2) * dt;  // ((a+b)/2) * Delta T
        double i_term = kI * err_sum;

        // D Term
        double change_rate = (error - last_error) / dt;
        double d_term = kD * change_rate;

        // All together now
        control_update = p_term + i_term + d_term;

        last_time = time;
        last_error = error;

        return control_update;
    }
}

