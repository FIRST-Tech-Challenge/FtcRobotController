package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import kotlin.random.Random;
import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.DriveShim;
import org.rowlandhall.meepmeep.roadrunner.SampleMecanumDrive;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.Scanner;

public class MeepMeepTesting {

    private static final Pose2d gameStartPose = new Pose2d(35, 60, Math.toRadians(-90));
    private static final Pose2d pickUpPose = new Pose2d(27, 0, Math.toRadians(180));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f);
//                .addEntity(myBot)
//                .start();


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(gameStartPose)
                        .splineToLinearHeading(pickUpPose, Math.toRadians(180 + 90))
                        .build());

        DriveShim botDrive = myBot.getDrive();
        TrajectorySequence mySequence = botDrive.trajectorySequenceBuilder(gameStartPose)
                .lineToLinearHeading(pickUpPose)
                .build();

        Scanner scanner = new Scanner(System.in);
        int poseHeading = 0;
        int endHeading = 0;
        meepMeep.addEntity(myBot);
        while (true) {
            meepMeep.start();

            System.out.println("enter new pose heading:");
            poseHeading = scanner.nextInt();
            System.out.println("enter new end heading:");
            poseHeading = scanner.nextInt();
            myBot.followTrajectorySequence(mySequence);
            System.out.println("next cycle");
        }
    }

}