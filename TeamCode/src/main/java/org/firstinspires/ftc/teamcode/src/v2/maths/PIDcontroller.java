package org.firstinspires.ftc.teamcode.src.v2.maths;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDcontroller {
    //PID controller class

    private double integralSum,out,lastError;
    private double Kp, Kd, Ki, Kf, Kl, Kv, Ka, Kstatic;
    private final double KpS, KdS, KiS, KfS, KlS;
    private final ElapsedTime timer = new ElapsedTime();

    public PIDcontroller(double Kp, double Kd, double Ki, double Kf, double Kl) {
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.Kf = Kf;
        this.Kl = Kl;

        KpS = Kp;
        KdS = Kd;
        KiS = Ki;
        KfS = Kf;
        KlS = Kl;
    }

    //calculate
    public double pidOut(double error) {
        if (Math.abs(error) > 0) {
            //integral and derivative values
            double derivative = (error - lastError) / timer.seconds();
            integralSum += (error * timer.seconds());
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

}