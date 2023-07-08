package org.firstinspires.ftc.teamcode;

// proportional integral derivative controller.

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDcontroller {

    public double integralSum = 0;
    /* we need to tune this based on victor lobmbona:
    Raise P until it oscillates around the setpoint. Add D to cancel out the oscillation. Only add I if you need to
     */
    private static double Kd = 0;
    private static double Ki = 0;
    private static double Kp = 0;
    private static double Kf = 0;
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    public double PIDFcontroller(double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double out = (error *Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);

        return out;
    }



}
