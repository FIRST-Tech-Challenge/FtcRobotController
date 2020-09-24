package com.hfrobots.tnt.season1819;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

/**
 * Road Runner has the y-axis extend out the left side of the robot, and the x-axis
 * extends out the front
 *
 * TNT's code (and the FTC field coordinate system) has y extend out the front of the robot
 * and x out the right
 *
 * This class maps between TNT's concept of axes, and RoadRunner's
 */
public class TntPose2d {

    public static Pose2d toPose2d(double tntX, double tntY) {
        return new Pose2d( tntY /* this is Road Runner's x axis value, it is our Y */  ,
                            -tntX /* This is Road Runner's y axis value, it would be our X */);
    }

    public static Pose2d toPose2d(double tntX, double tntY, double headingInDegrees) {
        return new Pose2d( tntY, -tntY, Math.toRadians(headingInDegrees));
    }

    public static Vector2d toVector2d(double tntX, double tntY) {
        return new Vector2d( tntY, -tntX);
    }

}
