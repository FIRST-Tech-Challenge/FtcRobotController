package org.firstinspires.ftc.teamcode.maths;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDcontroller {
    //PID controller class

    private double Kp;
    private double Kd;
    private double Ki;
    private double Kf;
    private double Kl;

    private double integralSum = 0;
    private double out = 0;
    private double lastError = 0;

    private double Kv = 0;
    private double Ka = 0;
    private double Kstatic = 0;

    private final double KpS;
    private final double KdS;
    private final double KiS;
    private final double KfS;
    private final double KlS;

    private final ElapsedTime timer = new ElapsedTime();

    public PIDcontroller(double Kp, double Kd, double Ki, double Kf, double Kl) {
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.Kf = Kf;
        this.Kl = Kl;

        this.KpS = Kp;
        this.KdS = Kd;
        this.KiS = Ki;
        this.KfS = Kf;
        this.KlS = Kl;

        timer.reset();
    }

    //calculate
    public double pidOut(double error) {
        if (Math.abs(error) > 0) {
            double dt = timer.seconds();
            if (dt <= 0) {
                dt = 1e-3;
            }
            //integral and derivative values
            double derivative = (error - lastError) / dt;
            integralSum += error * dt;
            integralSum = Range.clip(integralSum, -Kl, Kl);
            //weight each term so that tuning makes a difference
            out = (Kp * error) + (Kd * derivative) + (Ki * integralSum) + (Kf * Math.signum(error));
            out /= 10;
            lastError = error;
            timer.reset();
        }
        return out;
    }

    public double ffOut(double error, double velocityTarget, double accelerationTarget) {
        return pidOut(error) + Kv * velocityTarget + Ka * accelerationTarget + Kstatic;
    }

    public void setPIDgains(double Kp, double Kd, double Ki, double Kf, double Kl) {
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.Kf = Kf;
        this.Kl = Kl;
    }

    public void setFFgains(double Kv, double Ka, double Kstatic) {
        this.Kv = Kv;
        this.Ka = Ka;
        this.Kstatic = Kstatic;
    }

    public void toDefault() {
        this.Kp = KpS;
        this.Kd = KdS;
        this.Ki = KiS;
        this.Kf = KfS;
        this.Kl = KlS;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        out = 0;
        timer.reset();
    }
}
