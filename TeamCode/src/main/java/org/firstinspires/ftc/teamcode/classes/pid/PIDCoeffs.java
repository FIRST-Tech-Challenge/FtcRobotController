package org.firstinspires.ftc.teamcode.classes.pid;

public class PIDCoeffs {
    public double Kp;
    public double Ki;
    public double Kd;


    public PIDCoeffs(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }
}