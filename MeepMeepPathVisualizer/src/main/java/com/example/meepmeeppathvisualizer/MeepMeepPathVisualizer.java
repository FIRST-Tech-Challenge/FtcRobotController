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
        int numCycles = 10;

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
                .setDimensions(13.5, 16)

                // () -> {} is the syntax for taking in a function, aka a lambda
                .followTrajectorySequence(drive -> {
                    TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);

                    for(int i = 0; i < numCycles; i++) {
                        builder.forward(15);
                        builder.waitSeconds(1.5);
                        builder.lineToLinearHeading(new Pose2d(10,-63, Math.toRadians(0)));
                        builder.forward(5);
                    }

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