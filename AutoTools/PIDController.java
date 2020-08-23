package org.firstinspires.ftc.teamcode.rework.AutoTools;

import org.firstinspires.ftc.teamcode.rework.Robot;

public class PIDController {
    public double P;
    public double I;
    public double D;
    Robot robot;
    double integral, prevError;
    public double scale;

    public PIDController(double P, double I, double D, Robot robot){
        this.P = P;
        this.I = I;
        this.D = D;
        this.robot = robot;
    }

    public void PID(double distance, double targetDistance){
        double error = targetDistance-distance;
        integral += error*0.004;
        double derivative = (error - prevError) / 0.004;
        scale = P*error + I*integral + D*derivative;
        robot.telemetryDump.addHeader("PID");
        robot.telemetryDump.addData("scale: ", scale);
    }

    public void PID(Point current, Point target){
        double error = Math.hypot(current.x-target.x,current.y-target.y);
        integral += error*0.004;
        double derivative = (error - prevError) / 0.004;
        scale = P*error + I*integral + D*derivative;
        robot.telemetryDump.addHeader("PID");
        robot.telemetryDump.addData("scale: ", scale);
    }
}
