package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.kV;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPOVVelocity;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import static java.lang.Double.NaN;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;

public class RFPoseSim {
    double lastUpdateTime = 0.0;
    Pose2d realAcceleration = new Pose2d(0,0,0);
    Pose2d rotAcceleration = new Pose2d(0,0,0);
    public RFPoseSim(){

    }
    public void getTargets(double pF, double pS, double headingAccel){
//        if(currentPose.vec().distTo(p_targetPose.vec())>2) {
//            currentPose = p_targetPose;
//        }
//        if(!Double.isNaN(p_targetVelo.getHeading())) {
//            currentVelocity = p_targetVelo;
//        }
//        lastUpdateTime = time;

    }
    public double reduce(double in){
        if(abs(in)>1)
        {
            in*= 1/abs(in);
        }
        return in;
    }
    public void updateSim(double pF, double pS, double pR){
        double diffTime = time-lastUpdateTime;
        pF = reduce(pF);
        pS = reduce(pS);
        pR = reduce(pR);
        rotAcceleration = new Pose2d((pF - currentPOVVelocity.getX()*kV)/kA, (pS - currentPOVVelocity.getY()*kV)/kA,
                (pR-currentVelocity.getHeading()*kV*0.5*TRACK_WIDTH)/(kA*0.5*TRACK_WIDTH));
        realAcceleration = new Pose2d(rotAcceleration.vec().rotated(currentPose.getHeading()), rotAcceleration.getHeading());
        Pose2d newVelocity = currentVelocity.plus(realAcceleration.times(diffTime));
        Pose2d avgVelocity = newVelocity.plus(currentVelocity).times(0.5);
        packet.put("rotAcceleration", rotAcceleration);
        packet.put("realAcceleration",realAcceleration);
        currentVelocity = newVelocity;
        currentPOVVelocity = new Pose2d(newVelocity.vec().rotated(-currentPose.getHeading()), newVelocity.getHeading());
        lastUpdateTime=time;
        currentPose = currentPose.plus(avgVelocity.times(diffTime));
        Canvas fieldOverlay = packet.fieldOverlay();
        if(currentPose!=null) {
            fieldOverlay.setStrokeWidth(1);
            fieldOverlay.setStroke("#4CAF50");
            DashboardUtil.drawRobot(fieldOverlay, currentPose);

        }
    }
}
