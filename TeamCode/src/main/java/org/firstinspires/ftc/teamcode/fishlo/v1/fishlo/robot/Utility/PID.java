package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot.Utility;

public class PID {

    private double Kp, Kd, Ki;
    private double p_error, d_error, i_error;
    private double error = 0;
    private double prev_error;
    private double loop_time;

    public PID(double proportional, double integral, double derivative) {
        Kp = proportional;
        Kd = derivative;
        Ki = integral;
    }

    public void setError(double err) {
        prev_error = error;
        error = err;
    }

    public void setLoopTime(double time) {
        loop_time = time;
    }

    public void proportionalError() {
        p_error = error;
    }

    public void derivativeError() {
        d_error = (error - prev_error)/loop_time;
    }

    public void integralError() {
        i_error = i_error + (error*loop_time);
    }

    public double getSetValue() {
        return (p_error*Kp) + (d_error*Kd) + (i_error*Ki);
    }
}
