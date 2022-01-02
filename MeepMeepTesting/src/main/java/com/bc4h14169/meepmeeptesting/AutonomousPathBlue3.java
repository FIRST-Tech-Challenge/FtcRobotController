package com.bc4h14169.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutonomousPathBlue3 {
    public static void main(String[] args){

        /*
        Steps for Autonomous:
        1) Move to the right
        2) Spin the carousel
        3) Move forward to the end of the blue square
        4) Move left to the Shipping Hub
        5) Move diagonally (back right) and then strafe along back wall to park in the warehouse
        */


        MeepMeep meepMeep = new MeepMeep(800);

        //Setup BotEntity's dimensions and constraints with our robot's parameters
        //blue path 3 spins the duck and parks in the warehouse
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(16.5, 13.25)
                //Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(41.065033847887705,41.065033847887705, Math.toRadians(100), Math.toRadians(180), 13.2435)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-36,60, Math.toRadians(270)))
                                    .splineToLinearHeading(new Pose2d(-63, 60, Math.toRadians(245)),Math.toRadians(180))
                                    .strafeTo(new Vector2d(-60, 22))
                                    .splineToLinearHeading(new Pose2d(-35, 22, Math.toRadians(0)),Math.toRadians(90))
                                    .splineToLinearHeading(new Pose2d(-60, 33, Math.toRadians(0)),Math.toRadians(90))
                                    .splineToLinearHeading(new Pose2d(-33, 60, Math.toRadians(270)),Math.toRadians(0))
                                    .strafeTo(new Vector2d(44, 60))
                                    .build()
                    );
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start();
    }
}
