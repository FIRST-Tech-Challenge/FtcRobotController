package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class RFWaypoint {
    private Pose2d target;
    private double endTangent;
    private double endVelocity;
    private int definedness;

    public RFWaypoint(Pose2d p_target, double p_endTangent, double p_endVelocity, int definedness) {
        createRFWaypoint(p_target, p_endTangent, p_endVelocity, definedness);
    }

    public RFWaypoint(Pose2d p_target, double p_endTangent, double p_endVelocity) {
        createRFWaypoint(p_target, p_endTangent, p_endVelocity, 3);
    }

    public RFWaypoint(Pose2d p_target, double p_endTangent) {
        createRFWaypoint(p_target, p_endTangent, 1000, 2);
    }

    public RFWaypoint(Pose2d p_target) {
        createRFWaypoint(p_target, 1000, 1000, 1);
    }

    public RFWaypoint(Vector2d p_target) {
        createRFWaypoint(new Pose2d(p_target, 1000), 1000, 1000, 0);
    }

    public void createRFWaypoint(Pose2d p_target, double p_endTangent, double p_endVelocity, int p_definedness) {
        target = p_target;
        endTangent = p_endTangent;
        endVelocity = p_endVelocity;
        definedness = p_definedness;
    }

    public void changeTo(RFWaypoint p_newWaypoint) {
        target = p_newWaypoint.target;
        endTangent = p_newWaypoint.endTangent;
        endVelocity = p_newWaypoint.endVelocity;
        definedness = p_newWaypoint.definedness;
    }

    public Pose2d getTarget() {
        return target;
    }

    public void setTarget(Pose2d p_target) {
        target = p_target;
    }

    public double getEndTangent() {
        return endTangent;
    }

    public void setEndTangent(double p_endTangent) {
        endTangent = p_endTangent;
    }

    public Vector2d getEndVelocityVec(){
        packet.put("endTangentCalc", endTangent);
        packet.put("endVeloCalc", endVelocity);
        return new Vector2d(cos(endTangent)*endVelocity, sin(endTangent)*endVelocity);
    }

    public double getEndVelocity() {
        return endVelocity;
    }

    public void setEndVelocity(double p_endVelocity) {
        endVelocity = p_endVelocity;
    }

    public int getDefinedness() {
        return definedness;
    }
}
