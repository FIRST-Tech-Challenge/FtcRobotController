package org.bluebananas.ftc.roadrunneractions.TrajectoryActionBuilders;

import com.acmerobotics.roadrunner.Pose2d;

public class SpecimenPose {
    public final static Pose2d pose_basket_init_old = new Pose2d(31, 61.875, Math.toRadians(180));
    public final static Pose2d pose_init = new Pose2d (47.363,59.42, Math.toRadians(-135));
    public final static Pose2d pose_drop = new Pose2d(55.174, 52.521, Math.toRadians(-135));
    public final static Pose2d pose_inner_sample = new Pose2d(50.5, 49, Math.toRadians(-93.5)); //inner closest to wall
    public final static Pose2d pose_middle_sample = new Pose2d(56.5, 49.5, Math.toRadians(-85));
    //public final static Pose2d pose_outer_sample = new Pose2d(60.321, 51.18, Math.toRadians(-71.267));
    public final static Pose2d pose_outer_sample = new Pose2d(60, 48.5, Math.toRadians(-71));; //outer closest to wall
    public final static Pose2d pose_park = new Pose2d(-38, 60, Math.toRadians(180));
    public final static Pose2d pose_fifth_sample = new Pose2d(31, 61.875, Math.toRadians(180));
}
