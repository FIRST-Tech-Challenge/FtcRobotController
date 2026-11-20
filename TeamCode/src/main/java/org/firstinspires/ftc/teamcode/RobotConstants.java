package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

public class RobotConstants {


    public double intakeMotorSpeed = 1;
    public double outtakeMotorSpeed = 1150;
    public double allowedOuttakeSpeedError = 25;
    public double transferMotorSpeed = 0.55;
    public double stopperDownPosition = 0.12;
    public double stopperUpPosition = 0.07;
    public double hiveCentricRotationSpeed = 0.496;




    // Start Wall
    public Pose startPoseRedWall = new Pose(8.422, 109.323, Math.toRadians(0));
    public Pose startPoseBlueWall = startPoseRedWall;

    // Start Corner
    public Pose startPoseRedCorner = new Pose(118.638, 129.298, Math.toRadians(216));
    public Pose startPoseBlueCorner = startPoseRedCorner;

    // Score
    public Pose scorePoseRed = new Pose(99.104, 92.945, Math.toRadians(45));
    public Pose scorePoseBlue = scorePoseRed;

    // End
    public Pose endPoseRed = new Pose(95.694, 61.789, Math.toRadians(180));
    public Pose endPoseBlue = endPoseRed;


}