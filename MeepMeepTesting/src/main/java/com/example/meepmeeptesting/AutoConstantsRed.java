package com.example.meepmeeptesting;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;

import java.util.Arrays;
import java.util.function.Function;
import java.util.function.Supplier;

public class AutoConstantsRed {
    public static class Away {
        public static Pose2d START = new Pose2d(36, -66, toRadians(90));
        public static Pose2d STACK = new Pose2d(60, -12, toRadians(0));

        public static Pose2d PARK_LEFT = new Pose2d(12, -36, toRadians(90));
        public static Pose2d PARK_MIDDLE = new Pose2d(36, -36, toRadians(-90));
        public static Pose2d PARK_RIGHT = new Pose2d(60, -36, toRadians(180));

        public static Pose2d W_JUNCTION = new Pose2d(27, -5, toRadians(120));
        public static Pose2d S_JUNCTION = new Pose2d(4, -28, toRadians(135));

        //right facing between
        public static Pose2d W_JUNCTION_TO_BETWEEN = new Pose2d(34, -15, toRadians(0));
        //left facing between
        public static Pose2d STACK_TO_BETWEEN = new Pose2d(37, -12, toRadians(180));

        public static Pose2d LOW_TO_BETWEEN = new Pose2d(33,-60,toRadians(165));

        public static Pose2d LOW_JUNCTION_LEFT = new Pose2d(26,-52, toRadians(120));
        public static Pose2d LOW_JUNCTION_RIGHT = new Pose2d(48,-24, toRadians(60));


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
        public static Supplier<Trajectory>

                START_TO_W_JUNCTION =
                () -> function.apply(START).splineTo(W_JUNCTION.vec(), W_JUNCTION.getHeading()).build(),
        //START_TO_SOUTH_JUNCTION =
        // ()-> function.apply(START).lineToLinearHeading(SOUTH_JUNCTION).build(),
        W_JUNCTION_TO_STACK =
                () -> function.apply(W_JUNCTION).lineToLinearHeading(STACK).build(),
        //SOUTH_JUNCTION_TO_STACK = b->b.apply(JUNCTION).lineToLinearHeading(STACK).build(),
        STACK_TO_W_JUNCTION =
                () -> function.apply(STACK).lineToLinearHeading(W_JUNCTION).build(),
        //STACK_TO_SOUTH_JUNCTION =
        // ()->function.apply(STACK).lineToLinearHeading(JUNCTION).build(),
        W_JUNCTION_TO_JUNCTION_TO_BETWEEN =
                () -> function.apply(W_JUNCTION).lineToLinearHeading(W_JUNCTION_TO_BETWEEN).build(),

        STACK_TO_STACK_TO_BETWEEN =
                () -> function.apply(STACK).lineToLinearHeading(STACK_TO_BETWEEN).build(),
                BETWEEN_TO_STACK =
                        () -> function.apply(W_JUNCTION_TO_BETWEEN).lineToLinearHeading(STACK).build(),
                BETWEEN_TO_W_JUNCTION =
                        () -> function.apply(STACK_TO_BETWEEN).lineToLinearHeading(W_JUNCTION).build(),

        W_JUNCTION_TO_PARK_LEFT =
                () -> function.apply(W_JUNCTION).lineToLinearHeading(PARK_LEFT).build(),
                W_JUNCTION_TO_PARK_MIDDLE =
                        () -> function.apply(W_JUNCTION).lineToLinearHeading(PARK_MIDDLE).build(),
                W_JUNCTION_TO_PARK_RIGHT =
                        () -> function.apply(W_JUNCTION).lineToLinearHeading(PARK_RIGHT).build(),


        //SOUTH_JUNCTION_TO_PARK_LEFT =
        // ()->function.apply(JUNCTION).lineToLinearHeading(PARK_LEFT).build()
        //SOUTH_JUNCTION_TO_PARK_MIDDLE =
        // ()->function.apply(JUNCTION).lineToLinearHeading(PARK_MIDDLE).build()
        //SOUTH_JUNCTION_TO_PARK_RIGHT =
        // ()->function.apply(JUNCTION).lineToLinearHeading(PARK_RIGHT).build()


        //Left Low Junction
        START_TO_LEFT_LOW =
                () -> function.apply(START).lineToLinearHeading(LOW_JUNCTION_LEFT).build(),

        LEFT_LOW_TO_BETWEEN_LEFT =
                () -> function.apply(LOW_JUNCTION_LEFT).lineToLinearHeading(LOW_TO_BETWEEN).build(),
        BETWEEN_TO_PARK_LEFT =
                () -> function.apply(LOW_TO_BETWEEN).splineTo(PARK_LEFT.vec(),PARK_LEFT.getHeading()).build(),

        LEFT_LOW_TO_PARK_MIDDLE =
                () -> function.apply(LOW_JUNCTION_LEFT).lineToLinearHeading(PARK_MIDDLE).build(),

        LEFT_LOW_TO_BETWEEN_RIGHT =
                () -> function.apply(LOW_JUNCTION_LEFT).lineToLinearHeading(PARK_MIDDLE).build(),

        //Right Low Junction
        START_TO_RIGHT_LOW =
                () -> function.apply(START).splineTo(LOW_JUNCTION_RIGHT.vec(), LOW_JUNCTION_RIGHT.getHeading()).build();


    }

    public static class Home {
        public static Pose2d START = new Pose2d(-36, 66, toRadians(-90));
        public static Pose2d STACK = new Pose2d(-62, 12, toRadians(180));
        public static Pose2d PARK_LEFT = new Pose2d(-60, -36, toRadians(0));
        public static Pose2d PARK_MIDDLE = new Pose2d(-36, 36, toRadians(-90));
        public static Pose2d PARK_RIGHT = new Pose2d(-12, -36, toRadians(-90));
        public static Pose2d E_JUNCTION = new Pose2d(-28, -4, toRadians(13));
        public static Pose2d S_JUNCTION = new Pose2d(-4, -28, toRadians(45));
        // between goes backward while rotating
        public static Pose2d BETWEEN = new Pose2d(-47, 12, toRadians(180));
        public static Pose2d BETWEEN2 = new Pose2d(-34, 12, toRadians(-30));
        public static Pose2d BETWEEN3 = new Pose2d(-44, -27, toRadians(30));
        public static Pose2d W_JUNCTION = new Pose2d(-30, 4, toRadians(-30));

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
        public static Supplier<Trajectory>


                START_TO_W_JUNCTION =
                () -> function.apply(START)
//                        .splineTo(W_JUNCTION.vec(), W_JUNCTION.getHeading())
                        .splineTo(W_JUNCTION.vec(), Math.toRadians(-50))
                        .build(),
        //START_TO_S_JUNCTION=
        //   () -> function.apply(START).lineToLinearHeading().build()
        W_TO_STACK =
                () -> function.apply(E_JUNCTION)
                        .lineToLinearHeading(STACK)
//                            .splineTo(E_JUNCTION.vec(), E_JUNCTION.getHeading()
                        .build(),
                STACK_TO_W_JUNCTION =
                        () -> function.apply(STACK)
                                .lineToLinearHeading(W_JUNCTION)
                                .build(),
        //STACK_TO_S_JUNCTION=
        //() -> function.apply(STACK).lineToLinearHeading().build(),
        W_JUNCTION_TO_PARK_LEFT =
                () -> function.apply(E_JUNCTION)
                        .lineToLinearHeading(PARK_LEFT)
                        .build(),
                W_JUNCTION_TO_PARK_RIGHT =
                        () -> function.apply(E_JUNCTION)
                                .lineToLinearHeading(PARK_RIGHT)
                                .build(),
                W_JUNCTION_TO_PARK_MIDDLE =
                        () -> function.apply(W_JUNCTION)
                                .splineTo(PARK_MIDDLE.vec(), Math.toRadians(-90))
                                .build(),
                S_JUNCTION_TO_PARK_LEFT =
                        () -> function.apply(E_JUNCTION)
                                .lineToLinearHeading(PARK_LEFT)
                                .build(),
                S_JUNCTION_TO_PARK_RIGHT =
                        () -> function.apply(E_JUNCTION)
                                .lineToLinearHeading(PARK_RIGHT)
                                .build(),
                S_JUNCTION_TO_PARK_MIDDLE =
                        () -> function.apply(E_JUNCTION)
                                .lineToLinearHeading(PARK_MIDDLE)
                                .build(),
                E_JUNCTION_TO_BETWEEN =
                        () -> function.apply(E_JUNCTION)
                                .lineToLinearHeading(BETWEEN)
                                .build(),
                W_JUNCTION_TO_BETWEEN =
                        () -> function.apply(W_JUNCTION)
                                .lineToLinearHeading(BETWEEN)
                                .build(),
                BETWEEN_TO_STACK =
                        () -> function.apply(BETWEEN)
                                .lineToLinearHeading(STACK)
                                .build(),
                STACK_TO_BETWEEN =
                        () -> function.apply(STACK)
                                .lineToLinearHeading(BETWEEN2)
                                .build(),
                BETWEEN_TO_W_JUNCTION =
                        () -> function.apply(BETWEEN2)
                                .lineToLinearHeading(W_JUNCTION)
                                .build(),
                START_TO_BETWEEN3 =
                        () -> function.apply(START)
                                .lineToLinearHeading(BETWEEN3)
                                .build(),
                BETWEEN3_TO_W_JUNCTION =
                        () -> function.apply(BETWEEN3)
                                .lineToLinearHeading(W_JUNCTION)
                                .build();
    }
}