package org.firstinspires.ftc.teamcode.roadrunner.secret;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Waypoint {
    public Pose2d getPose() {
        return pose;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public double getLookAhead() {
        return lookAhead;
    }

    public void setLookAhead(double lookAhead) {
        this.lookAhead = lookAhead;
    }


    Pose2d pose;
    double lookAhead;

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    double distance=0;

    public double getVelocity() {
        return velocity;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    double velocity = 0;
    public Waypoint(Pose2d pose, double lookAheadDistance){
        this.pose = pose;
        lookAhead = lookAheadDistance;
    }

}
