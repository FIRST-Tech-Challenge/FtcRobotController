package org.firstinspires.ftc.teamcode.commandBased;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.commandBased.classes.Pose2dSpline;

@Config
public class AutoLConstants {

    //locations
    public static Pose2d START_POSE = new Pose2d(35, 65, toRad(-90));
    public static Pose2d STACK_POSE = new Pose2d(62.75, 12.5, toRad(0));
    public static Pose2d MED_POSE = new Pose2d(36.5, 27.75, toRad(0));


    //medium
    public static Pose2dSpline INITIAL_SCORE_FIRST_MED = new Pose2dSpline(new Pose2d(36, 46, toRad(-90)), toRad(-90));
    public static Pose2dSpline INITIAL_SCORE_SECOND_MED = new Pose2dSpline(new Pose2d(33.75, 23, toRad(0)), toRad(-90));
    public static Pose2d       INITIAL_STACK_FIRST_MED = new Pose2d(35, 18, toRad(0));
    public static Pose2dSpline INITIAL_STACK_SECOND_MED = new Pose2dSpline(STACK_POSE, toRad(0));

    public static Pose2d       STACK_MED_FIRST = new Pose2d(35, 21, toRad(0));
    public static Pose2dSpline STACK_MED_SECOND = new Pose2dSpline(STACK_POSE, toRad(0));

    public static Pose2d       MED_FIRST = new Pose2d(48, 13, toRad(0));
    public static Pose2dSpline MED_SECOND = new Pose2dSpline(MED_POSE, toRad(90));

    //park
    public static Pose2d       PARK_L_FIRST = new Pose2d(35, 18, toRad(0));
    public static Pose2dSpline PARK_L_SECOND = new Pose2dSpline(new Pose2d(61, 14, toRad(0)), toRad(0));

    public static Pose2d       PARK_M_FIRST = new Pose2d(31.5, 10, toRad(0));

    public static Pose2d       PARK_R_FIRST = new Pose2d(35, 18, toRad(0));
    public static Pose2dSpline PARK_R_SECOND = new Pose2dSpline(new Pose2d(13, 14, toRad(0)), toRad(0));



    //13.5+
    public static double X_DRIFT_14 = 0.25;
    public static double Y_DRIFT_14 = 1.125;

    //13-13.5
    public static double X_DRIFT_13_5 = 0;
    public static double Y_DRIFT_13_5 = 1;

    //12.75-13
    public static double X_DRIFT_13 = 0;
    public static double Y_DRIFT_13 = 0.875;

    //12.5-12.75
    public static double X_DRIFT_12_75 = 0;
    public static double Y_DRIFT_12_75 = 0.75;

    //x-12.5
    public static double X_DRIFT_12_5 = 0.125;
    public static double Y_DRIFT_12_5 = 0.875;


    private static double toRad(double deg) {
        return Math.toRadians(deg);
    }

}
