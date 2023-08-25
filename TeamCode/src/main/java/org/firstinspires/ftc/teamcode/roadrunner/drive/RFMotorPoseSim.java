package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPos;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentTickPos;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentAcceleration;

import static java.lang.Double.NaN;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;

public class RFMotorPoseSim {
    double lastUpdateTime = 0.0;
    public void getTargets(double p_targetPos, double p_tickPos, double p_targetVelo, double p_targetAccel){
//        if(currentPose.vec().distTo(p_targetPose.vec())>2) {
        currentPos = p_targetPos;
//        }
        currentTickPos = p_tickPos;
        currentVelocity = p_targetVelo;
        currentAcceleration = p_targetAccel;
        lastUpdateTime = time;
    }
    public void updateSim(){
        Pose2d targetPose = new Pose2d(currentPos, 0, 0);
        lastUpdateTime=time;
        Canvas fieldOverlay = packet.fieldOverlay();
        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.setStroke("#4CAF50");
        DashboardUtil.drawRobot(fieldOverlay, targetPose);
        }
    }
