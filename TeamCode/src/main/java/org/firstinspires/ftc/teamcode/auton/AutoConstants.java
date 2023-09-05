package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import org.firstinspires.ftc.teamcode.drivePose.SampleMecanumDrive;

@Config
public class AutoConstants {
    public static double stack2Offset = 1;

    // BLUE LEFT Q1 (+, +)
    // BLUE RIGHT Q2 (-, +)
    // RED LEFT Q3 (-, -)
    // REF RIGHT Q4 (-, +)

    // Cycle parking constraints
    public static final TrajectoryVelocityConstraint PARK_VEL = SampleMecanumDrive.getVelocityConstraint(45, 5.578780276476597, 13.91);
    public static final TrajectoryAccelerationConstraint PARK_ACCEL = SampleMecanumDrive.getAccelerationConstraint(45);

    // GLOBAL
    public static double START_X = 36;
    public static double START_Y = 70.5 - (13.3858/2);
    public static double START_HEADING = Math.toRadians(270);

    public static double[] STACK_SLIDES_POSITIONS = {168, 125, 80, 45, 0};

    // LEFT PARKING VALUES
    public static double LPL_X = 57;
    public static double LPM_X = 36;
    public static double LPR_X = 12;
    // RIGHT PARKING VALUES
    public static double RPL_X = -LPR_X - 3;
    public static double RPM_X = -LPM_X;
    public static double RPR_X = -LPL_X - 3.5;

    public static double PARK_Y = 12;
    public static double PARK_HEADING = Math.toRadians(270);
    public static double RIGHT_PARK_HEADING = Math.toRadians(260);

    // BASIC PARKING
    public static double FORWARD_DIST = 30;
    public static double LATERAL_DIST = 24;

    // LEFT
    public static double L_SCORE_X = 37.35; //37.2 good on blue
    public static double L_SCORE_Y = 12;
    public static double L_SCORE_HEADING = Math.toRadians(180);

    public static double L_SCORE_MID_X = 37.2;
    public static double L_SCORE_NODUNK_X = 31.7;
    public static double L_SCORE_NODUNK_Y = 10.4;

    public static double L_STACK_X = 54.6;
    public static double L_STACK_Y = 12;
    public static double L_STACK_HEADING = Math.toRadians(180);

    public static final Pose2d L_START = new Pose2d(0.0, 0.0, 0.0);
//    public static final Pose2d L_START = new Pose2d(START_X, START_Y, START_HEADING);
    public static final Vector2d L_STACK = new Vector2d(L_STACK_X, L_STACK_Y);

    public static final Pose2d L_SCORE_POSE = new Pose2d(10.0, 0.0, 0.0);
//    public static final Pose2d L_SCORE_POSE = new Pose2d(L_SCORE_X, L_SCORE_Y, L_SCORE_HEADING);
    public static final Vector2d L_SCORE_VECTOR = new Vector2d(20.0, 0.0);
//    public static final Vector2d L_SCORE_VECTOR = new Vector2d(L_SCORE_X, L_SCORE_Y);

    public static final Pose2d L_SCORE_MID_POSE = new Pose2d(L_SCORE_MID_X + .55, L_SCORE_Y, L_SCORE_HEADING);
    public static final Vector2d L_SCORE_MID_VECTOR = new Vector2d(L_SCORE_MID_X, L_SCORE_Y);

    public static final Pose2d L_SCORE_NODUNK_POSE = new Pose2d(L_SCORE_NODUNK_X, L_SCORE_NODUNK_Y, L_SCORE_HEADING);
    public static final Vector2d L_SCORE_NODUNK_VECTOR = new Vector2d(L_SCORE_NODUNK_X, L_SCORE_NODUNK_Y);

    public static double R_SCORE_X = -41.9;
    public static double R_SCORE_Y = 14;
    public static double R_SCORE_HEADING = Math.toRadians(-10);

    public static double R_SCORE_MID_X = -42.35;

    public static double R_STACK_X = -60;
    public static double R_STACK_Y = 14;
    public static double R_STACK_HEADING = Math.toRadians(0);

    public static final Pose2d R_START = new Pose2d(-START_X, START_Y, START_HEADING);
    public static final Vector2d R_STACK = new Vector2d(R_STACK_X, R_STACK_Y);

    public static final Pose2d R_SCORE_POSE = new Pose2d(R_SCORE_X - .4, R_SCORE_Y, R_SCORE_HEADING);
    public static final Vector2d R_SCORE_VECTOR = new Vector2d(R_SCORE_X, R_SCORE_Y);

    public static final Pose2d R_SCORE_MID_POSE = new Pose2d(R_SCORE_MID_X - .2, R_SCORE_Y, R_SCORE_HEADING);
    public static final Vector2d R_SCORE_MID_VECTOR = new Vector2d(R_SCORE_MID_X, R_SCORE_Y);

    public static final Pose2d L_PARK_LEFT = new Pose2d(30.0, 0.0, 0.0);
//    public static final Pose2d L_PARK_LEFT = new Pose2d(LPL_X, PARK_Y, PARK_HEADING);
    public static final Pose2d L_PARK_MIDDLE = new Pose2d(LPM_X, PARK_Y, PARK_HEADING);
    public static final Pose2d L_PARK_RIGHT = new Pose2d(LPR_X, PARK_Y, PARK_HEADING);
    public static final Pose2d R_PARK_LEFT = new Pose2d(RPL_X, PARK_Y + 3, RIGHT_PARK_HEADING);
    public static final Pose2d R_PARK_MIDDLE = new Pose2d(RPM_X, PARK_Y + 3, RIGHT_PARK_HEADING);
    public static final Pose2d R_PARK_RIGHT = new Pose2d(RPR_X, PARK_Y + 3, RIGHT_PARK_HEADING);
}
