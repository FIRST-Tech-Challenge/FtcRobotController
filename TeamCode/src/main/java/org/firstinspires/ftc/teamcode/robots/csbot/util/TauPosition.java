package org.firstinspires.ftc.teamcode.robots.csbot.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.io.Serializable;

public class TauPosition implements Serializable {
    private static final long serialVersionUID = 1234L;
    private double chassisX;
    private double chassisY;
    private double chassisHeading;
    private double turretHeading;
    private int turretTicks;
    private long timestamp;
    public TauPosition() {
        chassisX = 0;
        chassisY = 0;
        chassisHeading = 0;
        turretHeading=0;
        timestamp = System.currentTimeMillis();
    }

    public TauPosition(Pose2d chassisPose, double turretHeading, int turretTicks) {
        this.chassisX = chassisPose.getX();
        this.chassisY = chassisPose.getY();
        this.chassisHeading = chassisPose.getHeading();
        this.turretHeading= turretHeading;
        this.turretTicks = turretTicks;
        timestamp = System.currentTimeMillis();
    }


    public void setPose(Pose2d pose){
        this.chassisX = pose.getX();
        this.chassisY = pose.getY();
        this.chassisHeading = pose.getHeading();
    }
    public void setTurretHeading(double heading) {
        turretHeading = heading;
    }
    public void setTurretTicks(int ticks) {
        turretTicks = ticks;
    }
    public void updateTime() { timestamp = System.currentTimeMillis(); }
    public Pose2d getPose(){
        return new Pose2d(chassisX, chassisY, chassisHeading);
    }
    public double getTurretHeading() { return turretHeading; }
    public int getTurretTicks() { return turretTicks; }
    public long getTimestamp() { return timestamp; }
}