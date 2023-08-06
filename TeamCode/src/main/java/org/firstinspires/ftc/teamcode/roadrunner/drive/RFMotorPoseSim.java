package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPos;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import static java.lang.Double.NaN;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;

public class RFMotorPoseSim {
    double lastUpdateTime = 0.0;
    public void getTargets(double p_targetPos, double p_targetVelo){
//        if(currentPose.vec().distTo(p_targetPose.vec())>2) {
        currentPos = p_targetPos;
//        }
        currentVelocity = p_targetVelo;

        lastUpdateTime = time;
    }
    public void updateSim(){
        Pose2d targetPose = new Pose2d(0, currentPos, 0);
        lastUpdateTime=time;
        Canvas fieldOverlay = packet.fieldOverlay();
        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.setStroke("#4CAF50");
        DashboardUtil.drawRobot(fieldOverlay, targetPose);
        }
    }
