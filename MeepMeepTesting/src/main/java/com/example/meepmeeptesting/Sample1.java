package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.DriveShim;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class Sample1 {
    public static TrajectorySequence myTrajectory(DriveShim drive) {
        Pose2d startPose = new Pose2d(0, 0, 0);
        return myTrajectory(drive, startPose);
    }

    public static TrajectorySequence myTrajectory(DriveShim drive, Pose2d startPose) {
        TrajectorySequenceBuilder ret;
        drive.setPoseEstimate(startPose);

        ret = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(15, 25), 0)
                .turn(Math.toRadians(90))
                .splineTo(new Vector2d(10, -10), 0)
                .waitSeconds(3)
                .turn(Math.toRadians(45))
                .forward(10)
                .strafeLeft(5)
                .turn(Math.toRadians(90))
                .strafeLeft(5)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(45)), 0)
        ;

        return ret.build();
    }

    public static TrajectorySequenceBuilder goToCorner(TrajectorySequenceBuilder x) {
        return x.splineToConstantHeading(new Vector2d(30, 30), 0);
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> myTrajectory(drive));


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(.2f)
                .addEntity(myBot)
                .start();
    }
}