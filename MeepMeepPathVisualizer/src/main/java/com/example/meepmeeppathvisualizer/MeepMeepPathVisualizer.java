package com.example.meepmeeppathvisualizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.Vector;

public class MeepMeepPathVisualizer {

    public static void main(String[] args){
        int numCycles = 3;

        MeepMeep MM = new MeepMeep(800,90);
        Pose2d startPose = new Pose2d(-12, -59, Math.toRadians(90)); // x, y, heading (angle in radians)

        // Creating bot
        RoadRunnerBotEntity bot = new DefaultBotBuilder(MM)
                .setStartPose(startPose)

                .setConstraints(
                        54, // Retrieves constants from the 'Drive Constants' file
                        54,
                        Math.toRadians(63.0254), //MAX_ANG_VEL
                        Math.toRadians(63.0254), //MAX_ANG_ACCEL
                        14.5
                )

                .setColorScheme(new ColorSchemeRedDark())

                // Dimensions of the bot
                .setDimensions(13, 16)

                // () -> {} is the syntax for taking in a function, aka a lambda
                // Assuming FRONT of the bot is the EXTAKE
                .followTrajectorySequence(drive -> {
                    int stepIncrement = 0;
                    TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);
                    builder.forward(15);

                    builder.waitSeconds(1.5);
                    for (int i = 0; i < numCycles; i++) {
                    builder.lineToLinearHeading(new Pose2d(10,-63.5, Math.toRadians(180)));
                    builder.back(25 + stepIncrement);
                    builder.waitSeconds(1);
                    builder.forward(25 + stepIncrement);
                    builder.lineToLinearHeading(new Pose2d(0,-42, Math.toRadians(125)));
                    builder.waitSeconds(1.5);
                    stepIncrement += 3;
                    }

                    builder.setReversed(true);
                    builder.splineToSplineHeading(new Pose2d(-59,-35), Math.toRadians(90));

                    return builder.build();
                });

        //(MeepMeep.Background.GRID_GRAY)
        MM.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)

                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                .addEntity(bot)
                .start();
        }

    }