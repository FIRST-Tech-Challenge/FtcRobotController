package org.firstinspires.ftc.teamcode.commandBased;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.commandBased.classes.Pose2dSpline;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

@Config
public class AutoConstants {

    public static Pose2d START_POSE_LEFT = new Pose2d(35, 65, Math.toRadians(-90));


    public static Pose2dSpline INITIAL_FIRST_MED_LEFT = new Pose2dSpline(new Pose2d(36, 46, toRad(-90)), toRad(-90));
    public static Pose2dSpline INITIAL_SECOND_MED_LEFT = new Pose2dSpline(new Pose2d(36, 23, toRad(0)), toRad(-90));
    public static Pose2d       INITIAL_THIRD_MED_LEFT = new Pose2d(35, 18, toRad(0));
    public static Pose2dSpline INITIAL_FOURTH_MED_LEFT = new Pose2dSpline(new Pose2d(63.5, 14.5, toRad(0)), toRad(0));

    public static Pose2d       STACK_FIRST = new Pose2d(35, 18, toRad(0));
    public static Pose2dSpline STACK_SECOND = new Pose2dSpline(new Pose2d(35, 18, toRad(0)), toRad(0));

    public static Pose2d       MED_FIRST = new Pose2d(48, 13, toRad(0));
    public static Pose2dSpline MED_SECOND = new Pose2dSpline(new Pose2d(36.5, 28.5, toRad(0)), toRad(90));

    public static Pose2d       PARK_L_FIRST_LEFT = new Pose2d(35, 18, toRad(0));
    public static Pose2dSpline PARK_L_SECOND_LEFT = new Pose2dSpline(new Pose2d(62, 14, toRad(0)), toRad(0));

    public static Pose2d       PARK_M_FIRST_LEFT = new Pose2d(34.5, 10, toRad(0));

    public static Pose2d       PARK_R_FIRST_LEFT = new Pose2d(35, 18, toRad(0));
    public static Pose2dSpline PARK_R_SECOND_LEFT = new Pose2dSpline(new Pose2d(9, 14, toRad(0)), toRad(0));


    public static double xIncrement = 0.25;
    public static double yIncrement = 1;

    private static double toRad(double deg) {
        return Math.toRadians(deg);
    }

}
