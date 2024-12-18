package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;

public class RotatedPose2d  {
    public static Pose2d rotate90deg(Pose2d pose2d){
        if(pose2d.position.x>=0 && pose2d.position.y>=0){
            return new Pose2d(pose2d.position.y,-pose2d.position.x,pose2d.heading.toDouble() - Math.toRadians(90));
        }
        if(pose2d.position.x<=0 && pose2d.position.y>=0){
            return new Pose2d(pose2d.position.y,-pose2d.position.x,pose2d.heading.toDouble() - Math.toRadians(90));
        }
        if(pose2d.position.x<=0 && pose2d.position.y<=0){
            return new Pose2d(pose2d.position.y,-pose2d.position.x,pose2d.heading.toDouble() - Math.toRadians(90));
        }
        if(pose2d.position.x>=0 && pose2d.position.y<=0){
            return new Pose2d(pose2d.position.y,-pose2d.position.x,pose2d.heading.toDouble() - Math.toRadians(90));
        }
        return new Pose2d(0,0,0);
    }
}
