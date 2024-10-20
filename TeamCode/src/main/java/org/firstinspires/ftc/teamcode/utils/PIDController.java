package org.firstinspires.ftc.teamcode.utils;

import android.annotation.SuppressLint;

public class PIDController {
    private double Kp;
    private double Ki;
    private double Kd;

    private double Kf;
    private double target;

    private double prevError = 0;
    private double integral = 0;
    private double prevTime = System.currentTimeMillis() / 1000.0;

    public PIDController(double Kp, double Ki, double Kd, double Kf) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.target = 0;
    }

    public double calculate(double currentPosition, double forwardFeed) {
        double currentTime = System.currentTimeMillis() / 1000.0; // current time in seconds
        double deltaTime = currentTime - prevTime; // time difference

        double error = this.target - currentPosition;
        integral += error * deltaTime;
        double derivative = (error - prevError) / deltaTime;

        double output = Kp * error + Ki * integral + Kd * derivative + Kf * forwardFeed;
        prevError = error;
        prevTime = currentTime;

        return output;
    }

    public void setTarget(double target){
        this.target = target;
    }

    public double getTarget(){
        return this.target;
    }

    //These can be used to adjust PID actively
    public void setKp(double kp){
        Kp = kp;
    }

    public void setKi(double ki){
        Ki = ki;
    }

    public void setKd(double kd){
        Kd = kd;
    }

    public void setKf(double kf) {Kf = kf;}

    public double[] getPIDValues(){
        return new double[] {Kp, Ki, Kd, Kf};
    }

    @SuppressLint("DefaultLocale")
    public String toString(){
        return String.format("Kp: %f\nKi: %f\nKd: %f\nKf: %f",Kp, Ki, Kd,Kf);
    }
}

