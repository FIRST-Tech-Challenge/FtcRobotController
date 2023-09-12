package com.example.meepmeeptesting;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.Arrays;
import java.util.function.Function;

import java.util.function.Supplier;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;


public class AutoConstantsBlue {
    // "Home" locations: (The side with the Red terminal)
    public static class Away {
        //        public static Pose2d START = new Pose2d(36, 66, toRadians(-90));
//        public static Pose2d STACK = new Pose2d(60, 12, toRadians(0));
//        public static Pose2d LEFT = new Pose2d(60, 36, toRadians(150));
//        public static Pose2d MIDDLE = new Pose2d(36, 36, toRadians(90));
//        public static Pose2d RIGHT = new Pose2d(12, 36, toRadians(90));
//        // These have "home/away" modifiers, because we want to stay on "our side" during auto
//        // 12 O'Clock is on the Blue side: Probably stay away
//        public static Pose2d N_JUNCTION = new Pose2d(18, -3, toRadians(-135));
//        // 3 O'Clock is on the Away side, so only use if we know our alliance partner won't be
//        // in the way
//        public static Pose2d E_JUNCTION = new Pose2d(-28, 4, toRadians(-45));
//        public static Pose2d S_JUNCTION = new Pose2d(-10, 30, toRadians(-135));
//        public static Pose2d W_JUNCTION = new Pose2d(27, 8, toRadians(-115));
        public static Pose2d START = new Pose2d(36, -66, toRadians(90));
        public static ConfigurablePose TELESTART = new ConfigurablePose(0, 0, toRadians(90));
        public static ConfigurablePose FORWARD_MOVE = new ConfigurablePose(0, 24, toRadians(90));
        public static ConfigurablePose BACKWARD_MOVE = new ConfigurablePose(0,24, toRadians(90));
        public static Pose2d LEFT_MOVE = new Pose2d(-24, 0, toRadians(90));
        public static Pose2d RIGHT_MOVE = new Pose2d(24, 0, toRadians(90));

        public static ConfigurablePose STACK = new ConfigurablePose(67, -14, toRadians(0));
        public static Pose2d LEFT = new Pose2d(15, -16,toRadians(90));
        public static Pose2d MIDDLE = new Pose2d(36, -16, toRadians(90));
        public static Pose2d RIGHT = new Pose2d(60, -17, toRadians(90));
        public static ConfigurablePose W_JUNCTION = new ConfigurablePose(35, -9, 2.4);
        public static ConfigurablePose BETWEEN_START_W_JUNCTION = new ConfigurablePose(42, -15, 2.4);
        public static ConfigurablePose BETWEEN_W_JUNCTION_STACK = new ConfigurablePose(35, -16, .4);
        public static ConfigurablePose BETWEEN_STACK_W_JUNCTION = new ConfigurablePose(36, -14, 2.4);
        public static Pose2d BETWEEN_START_LEFT = new Pose2d(15, -60, toRadians(90));
        public static Pose2d BETWEEN_START_RIGHT = new Pose2d(60, -60, toRadians(90));
        public static ConfigurablePose TERMINAL = new ConfigurablePose(61,-64,toRadians(180));

        // These are 'trajectory pieces' which should be named like this:
        // {STARTING_POSITION}_TO_{ENDING_POSITION}
        public static double MAX_VEL = 50;
        public static double MAX_ACCEL = 40;
        public static double MAX_ANG_VEL = Math.toRadians(200);
        public static double MAX_ANG_ACCEL = Math.toRadians(200);
        public static double TRACK_WIDTH = 9.5;

        public static MinVelocityConstraint MIN_VEL = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        ));
        public static ProfileAccelerationConstraint PROF_ACCEL = new ProfileAccelerationConstraint(MAX_ACCEL);
        public static Function<Pose2d, TrajectoryBuilder> function = pose -> new TrajectoryBuilder(pose, MIN_VEL, PROF_ACCEL);
        public static Supplier<Trajectory>
                START_TO_LEFT_PARK =
                        () -> function.apply(START)
                                //.lineToLinearHeading(TERMINAL.toPose())
                                .lineToLinearHeading(BETWEEN_START_LEFT)
                                .lineToLinearHeading(LEFT_MOVE)
                                .build(),
                START_TO_MIDDLE_PARK =
                        () -> function.apply(START)
                                //.lineToLinearHeading(TERMINAL.toPose())
                                //.lineToLinearHeading(START.toPose())
                                .lineToLinearHeading(MIDDLE)
                                .build(),
                START_TO_RIGHT_PARK =
                        () -> function.apply(START)
                                //.lineToLinearHeading(TERMINAL.toPose())
                                .lineToLinearHeading(BETWEEN_START_RIGHT)
                                .lineToLinearHeading(RIGHT_MOVE)
                                .build();

    }

    // Away locations:
    public static class Home {
        public static ConfigurablePose START = new ConfigurablePose(36, -66, toRadians(90));
        public static ConfigurablePose TELESTART = new ConfigurablePose(0, 0, toRadians(90));
        public static ConfigurablePose FORWARD_MOVE = new ConfigurablePose(0, 24, toRadians(90));
        public static ConfigurablePose BACKWARD_MOVE = new ConfigurablePose(0,24, toRadians(90));
        public static ConfigurablePose LEFT_MOVE = new ConfigurablePose(-24, 0, toRadians(90));
        public static ConfigurablePose RIGHT_MOVE = new ConfigurablePose(24, 0, toRadians(90));

        public static ConfigurablePose STACK = new ConfigurablePose(67, -14, toRadians(0));
        public static ConfigurablePose LEFT = new ConfigurablePose(15, -16,toRadians(90));
        public static ConfigurablePose MIDDLE = new ConfigurablePose(36, -16, toRadians(90));
        public static ConfigurablePose RIGHT = new ConfigurablePose(60, -17, toRadians(90));
        public static ConfigurablePose W_JUNCTION = new ConfigurablePose(35, -9, 2.4);
        public static ConfigurablePose BETWEEN_START_W_JUNCTION = new ConfigurablePose(42, -15, 2.4);
        public static ConfigurablePose BETWEEN_W_JUNCTION_STACK = new ConfigurablePose(35, -16, .4);
        public static ConfigurablePose BETWEEN_STACK_W_JUNCTION = new ConfigurablePose(36, -14, 2.4);
        public static ConfigurablePose BETWEEN_START_LEFT = new ConfigurablePose(15, -60, toRadians(90));
        public static ConfigurablePose BETWEEN_START_RIGHT = new ConfigurablePose(60, -60, toRadians(90));
        public static ConfigurablePose TERMINAL = new ConfigurablePose(61,-64,toRadians(180));
        // These are 'trajectory pieces' which should be named like this:
        // {STARTING_POSITION}_TO_{ENDING_POSITION}
        public static double MAX_VEL = 50;
        public static double MAX_ACCEL = 40;
        public static double MAX_ANG_VEL = Math.toRadians(180);
        public static double MAX_ANG_ACCEL = Math.toRadians(120);
        public static double TRACK_WIDTH = 9.5;

        public static MinVelocityConstraint MIN_VEL = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        ));
        public static ProfileAccelerationConstraint PROF_ACCEL = new ProfileAccelerationConstraint(MAX_ACCEL);
        public static Function<Pose2d, TrajectoryBuilder> function = pose -> new TrajectoryBuilder(pose, MIN_VEL, PROF_ACCEL);
   //     public static Supplier<Trajectory>



    }
}
