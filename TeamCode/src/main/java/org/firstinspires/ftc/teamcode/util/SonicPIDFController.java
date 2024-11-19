package org.firstinspires.ftc.teamcode.util;

public class SonicPIDFController {
    double kp, ki, kd, kf;

    double integral = 0;
    double previousError = 0;

    long lastTimeMillis;

    public SonicPIDFController(double kp, double ki, double kd) {
        this(kp, ki, kd, 0);
    }

    public SonicPIDFController(double kp, double ki, double kd, double kf) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
        lastTimeMillis = System.currentTimeMillis();
    }

    public double calculatePIDAlgorithm(double error) {
        double deltaT = (System.currentTimeMillis() - lastTimeMillis) / 1000.0;
        integral += error * deltaT;
        double derivative = ((error - previousError) / deltaT);
        double proportional = error;
        double output = kp * proportional + ki * integral + kd * derivative + kf;
        previousError = error;
        lastTimeMillis = System.currentTimeMillis();
        return output;
    }

    public void resetIntegral() {
        integral = 0;
        lastTimeMillis = System.currentTimeMillis();
    }

    public void setPIDGains(double kp, double ki, double kd, double kf) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
    }

    public double getKp() {
        return kp;
    }

    public void setKp(double kp) {
        this.kp = kp;
    }

    public double getKi() {
        return ki;
    }

    public double getKf() {
        return kf;
    }

    public void setKi(double ki) {
        this.ki = ki;
    }

    public double getKd() {
        return kd;
    }

    public void setKd(double kd) {
        this.kd = kd;
    }

    public void setKf(double kf) {
        this.kf = kf;
    }
}