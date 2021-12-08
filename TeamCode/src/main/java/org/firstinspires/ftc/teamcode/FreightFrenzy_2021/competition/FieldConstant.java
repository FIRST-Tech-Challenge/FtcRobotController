package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import static java.lang.Math.toRadians;

public class FieldConstant {
    //Robot constants
    public static final double LENGTH = 17.5;
    public static final double WIDTH = 12.25;

    public static final Pose2d BLUE_PLATE = new Pose2d(-11.875, 23.75);
    public static final Pose2d RED_PLATE = new Pose2d(-11.875, -23.75);
    public static final double PLATE_RADIUS = 9; //in
    public static final Pose2d SHARED_HUB = new Pose2d(47.5, 0);
    public static final double SHARED_HUB_RADIUS = 9; //in

    public static final double PASSAGE_WIDTH = 13.7; //in

    //AUTO
    //Start Pose
    public static final Pose2d BLUE_DUCK_STARTING_POSE = new Pose2d(-41, 62.125, toRadians(90));
    public static final Pose2d RED_DUCK_STARTING_POSE = new Pose2d(-41, -62.125, toRadians(-90));
    public static final Pose2d BLUE_BARRIER_STARTING_POSE = new Pose2d(6.5, 62.125, toRadians(90));
    public static final Pose2d RED_BARRIER_STARTING_POSE = new Pose2d(6.5, -62.125, toRadians(-90));

    //TELE
    //Warehouse
    public static final Pose2d SHARED_RED_ENTER_POSE = new Pose2d(64.75, -36.125, toRadians(-90)); //-18 back(18.125)
    public static final Pose2d SHARED_BLUE_ENTER_POSE = new Pose2d(64.75, 36.125, toRadians(90)); //18 back(18.125)

    public static final Pose2d SHARED_RED_END_POSE = new Pose2d(60.473, -13.537, toRadians(313.781));
    public static final Pose2d SHARED_BLUE_END_POSE = new Pose2d(60.473, 13.537, toRadians(46.219));

}
