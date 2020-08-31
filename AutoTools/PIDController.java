package org.firstinspires.ftc.teamcode.rework.AutoTools;

import org.firstinspires.ftc.teamcode.rework.ModuleTools.TelemetryProvider;
import org.firstinspires.ftc.teamcode.rework.Robot;
import org.firstinspires.ftc.teamcode.rework.RobotTools.TelemetryDump;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class PIDController implements TelemetryProvider {
    public double P;
    public double I;
    public double D;
    Robot robot;
    double integral, prevError;
    public double scale;

    public PIDController(double P, double I, double D, Robot robot){
        TelemetryDump.registerProvider(this);
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
    }

    public void PID(Point current, Point target){
        double error = Math.hypot(current.x-target.x,current.y-target.y);
        integral += error*0.004;
        double derivative = (error - prevError) / 0.004;
        scale = P*error + I*integral + D*derivative;
    }

    @Override
    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("Scale: " + String.valueOf(this.scale));
        return data;
    }
}
