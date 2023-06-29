package org.firstinspires.ftc.teamcode.commandBased;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.commandBased.classes.Pose2dSpline;

@Config
public class AutoConstants {

    public static Pose2d START_POSE_LEFT = new Pose2d(35, 65, toRad(-90));
    public static Pose2d STACK_POSE_LEFT = new Pose2d(62.5, 12, toRad(0));
    public static Pose2d MED_POSE_LEFT = new Pose2d(36.5, 27.5, toRad(0));


    public static Pose2dSpline INITIAL_SCORE_FIRST_MED_LEFT = new Pose2dSpline(new Pose2d(36, 46, toRad(-90)), toRad(-90));
    public static Pose2dSpline INITIAL_SCORE_SECOND_MED_LEFT = new Pose2dSpline(new Pose2d(35, 23, toRad(0)), toRad(-90));
    public static Pose2d       INITIAL_STACK_FIRST_MED_LEFT = new Pose2d(35, 18, toRad(0));
    public static Pose2dSpline INITIAL_STACK_SECOND_MED_LEFT = new Pose2dSpline(STACK_POSE_LEFT, toRad(0));

    public static Pose2d       STACK_FIRST = new Pose2d(35, 21, toRad(0));
    public static Pose2dSpline STACK_SECOND = new Pose2dSpline(STACK_POSE_LEFT, toRad(0));

    public static Pose2d       MED_FIRST = new Pose2d(48, 13, toRad(0));
    public static Pose2dSpline MED_SECOND = new Pose2dSpline(MED_POSE_LEFT, toRad(90));

    public static Pose2d       PARK_L_FIRST_LEFT = new Pose2d(35, 18, toRad(0));
    public static Pose2dSpline PARK_L_SECOND_LEFT = new Pose2dSpline(new Pose2d(62, 14, toRad(0)), toRad(0));

    public static Pose2d       PARK_M_FIRST_LEFT = new Pose2d(34.5, 10, toRad(0));

    public static Pose2d       PARK_R_FIRST_LEFT = new Pose2d(35, 18, toRad(0));
    public static Pose2dSpline PARK_R_SECOND_LEFT = new Pose2dSpline(new Pose2d(9, 14, toRad(0)), toRad(0));


    public static double X_DRIFT = 0;
    public static double Y_DRIFT = 1;

    //13.5+
    public static double X_DRIFT_14 = -0.25;
    public static double Y_DRIFT_14 = 2;

    //13-13.5
    public static double X_DRIFT_13_5 = -0.25;
    public static double Y_DRIFT_13_5 = 1.5;

    //12.75-13
    public static double X_DRIFT_13 = -0.25;
    public static double Y_DRIFT_13 = 1.5;

    //12.5-12.75
    public static double X_DRIFT_12_75 = -0.25;
    public static double Y_DRIFT_12_75 = 2;

    //x-12.5
    public static double X_DRIFT_12_5 = 0;
    public static double Y_DRIFT_12_5 = 0.5;


    private static double toRad(double deg) {
        return Math.toRadians(deg);
    }

}
