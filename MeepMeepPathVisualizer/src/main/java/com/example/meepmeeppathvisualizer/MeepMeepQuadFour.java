package com.example.meepmeeppathvisualizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class MeepMeepQuadFour {
    public static void main(String[] args) {
        MeepMeep mm = new MeepMeep(800,120);
        new MeepMeepPersistence(mm);
        MeepMeepPersistence persist = new MeepMeepPersistence(mm);
        persist.restore();
        Pose2d startPose = new Pose2d(45, 15, Math.toRadians(96));

        // Creating bot
        RoadRunnerBotEntity bot = new DefaultBotBuilder(mm)
                .setStartPose(startPose)
                .setConstraints(
                        54,
                        54,
                        Math.toRadians(63.0254), //MAX_ANG_VEL
                        Math.toRadians(63.0254), //MAX_ANG_ACCEL
                        10
                )
                .setColorScheme(new ColorSchemeBlueDark())
                .setDimensions(13, 14)

                .followTrajectorySequence(drive -> {
                    TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);
                    builder.lineToLinearHeading(new Pose2d(45, -4, Math.toRadians(52)));
                    return builder.build();
                });


        mm.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(bot)
                .start();
    }

}