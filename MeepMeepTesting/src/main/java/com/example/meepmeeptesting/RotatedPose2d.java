package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;

public class RotatedPose2d  {
    public static Pose2d rotate90deg(Pose2d pose2d){
        return new Pose2d(pose2d.position.y,-pose2d.position.x,pose2d.heading.toDouble() - Math.toRadians(90));
//        return new Pose2d(pose2d.position.x,pose2d.position.y,pose2d.heading.toDouble());
    }
}