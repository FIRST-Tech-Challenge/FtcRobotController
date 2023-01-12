package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.function.Supplier;
import javax.sound.midi.Track;

public class MyClass {

    public static void main(String[] args) {

        int robotFB = 13;
        double dposition = 9.19;

        MeepMeep meepMeep = new MeepMeep(1000);

        Pose2d homePose = new Pose2d(35, -70.625, Math.toRadians(90));
        Pose2d midPose = new Pose2d(35, -12, Math.toRadians(90));
        Pose2d grabbingPose = new Pose2d(71.25 - robotFB, -12, Math.toRadians(180));
        Pose2d releasingPose = new Pose2d(24 + dposition, 0 - dposition, Math.toRadians(135));
        Pose2d parking1 = new Pose2d(12, -12, Math.toRadians(90));

        Pose2d parking2 = new Pose2d(36, -12, Math.toRadians(90));

        Pose2d parking3 = new Pose2d(60, -12, Math.toRadians(90));
        TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(
                Arrays.asList(new TranslationalVelocityConstraint(60), new AngularVelocityConstraint(60))
        );
        TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(100);

        Trajectory hm;
        hm = new TrajectoryBuilder(homePose, 2.2, velConstraint, accelConstraint)
                .lineToLinearHeading(midPose)
                .build();

        Trajectory home;
        home = new TrajectoryBuilder(midPose, 2.2, velConstraint, accelConstraint)
                .lineToLinearHeading(homePose)
                .build();

        Trajectory mr;
        mr = new TrajectoryBuilder(midPose, 2.2, velConstraint, accelConstraint)
                .lineToLinearHeading(releasingPose)
                .build();

        Trajectory rg;
        rg = new TrajectoryBuilder(releasingPose, 2.2, velConstraint, accelConstraint)
                 .lineToLinearHeading(grabbingPose)
                 .build();

        Trajectory gr;
        gr = new TrajectoryBuilder(grabbingPose, 2.2, velConstraint, accelConstraint)
                .lineToLinearHeading(releasingPose)
                .build();

        Trajectory rm;
        rm = new TrajectoryBuilder(releasingPose, 2.2, velConstraint, accelConstraint)
                .lineToLinearHeading(midPose)
                .build();

        Trajectory gm;
        gm = new TrajectoryBuilder(grabbingPose, 2.2, velConstraint, accelConstraint)
                .lineToLinearHeading(midPose)
                .build();

        Trajectory p1;
        p1 = new TrajectoryBuilder(midPose, 2.2, velConstraint, accelConstraint)
                .lineToLinearHeading(parking1)
                .build();

        Trajectory p2;
        p2 = new TrajectoryBuilder(midPose, 2.2, velConstraint, accelConstraint)
                .lineToLinearHeading(parking2)
                .build();

        Trajectory p3;
        p3 = new TrajectoryBuilder(midPose, 2.2, velConstraint, accelConstraint)
//                .lineToLinearHeading(midPose)
//                .lineToLinearHeading(parking2)
                .lineToLinearHeading(parking3)
                .build();

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13.11, 13.23)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11.81)
                .setStartPose(homePose)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(homePose)
                                .addTrajectory(hm)
                                .addTrajectory(mr)
                                .addTrajectory(rg)
                                .addTrajectory(gr)
                                .addTrajectory(rm)
                                .addTrajectory(p1)

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}