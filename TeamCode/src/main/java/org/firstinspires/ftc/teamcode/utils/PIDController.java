package org.firstinspires.ftc.teamcode.utils;

public class PIDController {
    private double Kp;
    private double Ki;
    private double Kd;
    private double target;

    private double prevError = 0;
    private double integral = 0;
    private double prevTime = System.currentTimeMillis() / 1000.0;

    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.target = 0;
    }

    public double calculate(double currentPosition) {
        double currentTime = System.currentTimeMillis() / 1000.0; // current time in seconds
        double deltaTime = currentTime - prevTime; // time difference

        double error = target - currentPosition;
        integral += error * deltaTime;
        double derivative = (error - prevError) / deltaTime;

        double output = Kp * error + Ki * integral + Kd * derivative;
        prevError = error;
        prevTime = currentTime;

        return output;
    }

    public void setTarget(double target){
        target = target;
    }

    public double getTarget(){
        return target;
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

    public double[] getPIDValues(){
        return new double[] {Kp, Ki, Kd};
    }

    public String toString(){
        return String.format("Kp: %f\nKi: %f\nKd: %f",Kp, Ki, Kd);
    }
}

