package org.firstinspires.ftc.teamcode.robots.taubot.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.io.Serializable;
import java.time.LocalTime;

public class TauPosition implements Serializable {
    private static final long serialVersionUID = 1234L;
    private Pose2d chassisPose;
    private double turretHeading;
    long timestamp;
    public TauPosition() {
        chassisPose = new Pose2d();
        turretHeading=0;
        timestamp = System.currentTimeMillis();
    }
    public void setPose(Pose2d pose){
        chassisPose =pose;
    }
    public void setTurretHeading(double heading) {
        turretHeading = heading;
    }
    public void updateTime() { timestamp = System.currentTimeMillis(); }
    public Pose2d getPose(){
        return chassisPose;
    }
    public double getTurretHeading() { return turretHeading; }
    public long getTimestamp() { return timestamp; }
}