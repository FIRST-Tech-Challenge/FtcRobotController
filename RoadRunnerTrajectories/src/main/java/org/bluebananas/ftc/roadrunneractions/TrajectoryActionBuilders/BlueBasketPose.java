package org.bluebananas.ftc.roadrunneractions.TrajectoryActionBuilders;

import com.acmerobotics.roadrunner.Pose2d;

public class BlueBasketPose {

    public final static Pose2d basket_init_old = new Pose2d(31, 61.875, Math.toRadians(180));
    public final static Pose2d init = new Pose2d(47, 59.5, Math.toRadians(-135));
    public final static Pose2d drop = new Pose2d(56.5, 55.875, Math.toRadians(-135));
    public final static Pose2d inner_sample = new Pose2d(50.5, 52, Math.toRadians(-96.5)); //inner closest to center
    public final static Pose2d middle_sample = new Pose2d(56.5, 52.5, Math.toRadians(-88));
    public final static Pose2d outer_sample = new Pose2d(59.25, 51.5, Math.toRadians(-73)); //outer closest to wall
    public final static Pose2d park = new Pose2d(-38, 60, Math.toRadians(180));
    public final static Pose2d fifth_sample = new Pose2d(31, 61.875, Math.toRadians(180));
    //fifth sample needs to be set at 8,61 parallel to alliance wall
    public final static Pose2d manual_teleop_init = drop; //This is the pose to use for a manual drive to and button press style reset
}