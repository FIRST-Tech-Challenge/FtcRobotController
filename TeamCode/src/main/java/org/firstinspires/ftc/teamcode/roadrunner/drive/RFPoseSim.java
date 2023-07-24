package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import static java.lang.Double.NaN;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;

public class RFPoseSim {
    double lastUpdateTime = 0.0;
    public RFPoseSim(){

    }
    public void getTargets(Pose2d p_targetPose, Pose2d p_targetVelo){
//        if(currentPose.vec().distTo(p_targetPose.vec())>2) {
            currentPose = p_targetPose;
//        }
        if(!Double.isNaN(p_targetVelo.getHeading())) {
            currentVelocity = p_targetVelo;
        }
        lastUpdateTime = time;
    }
    public void updateSim(){
        lastUpdateTime=time;
        Canvas fieldOverlay = packet.fieldOverlay();
        if(currentPose!=null) {
            fieldOverlay.setStrokeWidth(1);
            fieldOverlay.setStroke("#4CAF50");
            DashboardUtil.drawRobot(fieldOverlay, currentPose);
        }
    }
}
