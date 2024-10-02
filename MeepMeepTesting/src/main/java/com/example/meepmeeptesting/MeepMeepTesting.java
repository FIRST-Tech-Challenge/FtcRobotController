package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        //Each square is 23 by 23 pixel thingies

        RoadRunnerBotEntity redBotRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .setDimensions(17, 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(14, -61, Math.toRadians(90)))
                                //Outtakes preload
                                .lineToLinearHeading(new Pose2d(10, -61, Math.toRadians(90)))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(10, -35, Math.toRadians(90)))
                                .waitSeconds(1)
                                //Goes to cycle
                                .lineToLinearHeading(new Pose2d(32, -35, Math.toRadians(90)))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(55, -35, Math.toRadians(90)))
                                .waitSeconds(1)
                                .turn(Math.toRadians(-90))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(55, -55, Math.toRadians(0)))
                                .build()
                );

        RoadRunnerBotEntity redBotLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .setDimensions(17, 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-32, -61, Math.toRadians(90)))
                                .waitSeconds(10)
                                .build()
                );

        RoadRunnerBotEntity blueBotLeft = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .setDimensions(17, 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, 61, Math.toRadians(270)))
                                .waitSeconds(10)
                                .build()
                );

        RoadRunnerBotEntity blueBotRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 17)
                .setDimensions(17, 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-32, 61, Math.toRadians(270)))
                                //Outtakes preload
                                .lineToLinearHeading(new Pose2d(-10, 61, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-10, 35, Math.toRadians(270)))
                                .waitSeconds(10)
                                //Goes to cycle
                                .lineToLinearHeading(new Pose2d(-32, 35, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-60, 35, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(55, 35, Math.toRadians(270)))
                                .turn(Math.toRadians(90))
                                .lineToLinearHeading(new Pose2d(55, 55, Math.toRadians(0)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.GRID_GRAY)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redBotRight)
                .addEntity(blueBotRight)
                .start();
    }
}