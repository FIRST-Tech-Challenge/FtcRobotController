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
        Pose2d startPose = new Pose2d(-32, -23, Math.toRadians(180));

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
                   builder .setReversed(false);
                    builder.strafeRight(10);
                            builder.splineToLinearHeading(new Pose2d(-50, -10, Math.toRadians(180)), Math.toRadians(40));
                            builder.setReversed(false);
                    return builder.build();
                });


        mm.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(bot)
                .start();
    }

}