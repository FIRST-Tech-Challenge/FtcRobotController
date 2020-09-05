package org.firstinspires.ftc.teamcode.rework.Modules;

import android.os.SystemClock;

import org.firstinspires.ftc.teamcode.rework.ModuleTools.TelemetryProvider;
import org.firstinspires.ftc.teamcode.rework.Robot;

import java.util.ArrayList;

public class VelocityModule implements Module, TelemetryProvider {
    private boolean isOn;

    // in/s and rad/s
    public double xVel;
    public double yVel;
    public double angleVel;

    private long oldUpdateTime;
    private long currentUpdateTime;

    private double oldWorldX = 0;
    private double oldWorldY = 0;
    private double oldWorldAngle = 0;


    private Robot robot;

    public VelocityModule(Robot robot, boolean isOn) {
        robot.telemetryDump.registerProvider(this);
        this.robot = robot;
        this.isOn = isOn;
    }

    public void init() {
        oldUpdateTime = SystemClock.elapsedRealtime();
    }

    public void update() {
        currentUpdateTime = robot.currentTimeMilli;

        xVel = 1000 * (robot.odometryModule.worldX - oldWorldX) / (currentUpdateTime - oldUpdateTime);
        yVel = 1000 * (robot.odometryModule.worldY - oldWorldY) / (currentUpdateTime - oldUpdateTime);
        angleVel = 1000 * (robot.odometryModule.worldAngleRad - oldWorldAngle) / (currentUpdateTime - oldUpdateTime);

        oldWorldX = robot.odometryModule.worldX;
        oldWorldY = robot.odometryModule.worldY;
        oldWorldAngle = robot.odometryModule.worldAngleRad;
        oldUpdateTime = currentUpdateTime;
    }

    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("xVel: " + xVel);
        data.add("yVel: " + yVel);
        data.add("angleVel: "+ angleVel);
        return data;
    }

    public void fileDump(){

    }

    public boolean isOn(){
        return isOn;
    }
}
