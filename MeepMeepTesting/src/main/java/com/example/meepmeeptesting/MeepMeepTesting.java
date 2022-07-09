package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep mm = new MeepMeep(800);

        //depot bot
        RoadRunnerBotEntity depotBot = new DefaultBotBuilder(mm)
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(57.635630404559784, 38.7, 4.5836622, Math.toRadians(60), 14.2)
                .setDimensions(13.2,16.603)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, -63, Math.toRadians(-90)))
                                .addSpatialMarker(new Vector2d(-25,-39),() -> { })
                                .lineToSplineHeading(new Pose2d(-22,-40,Math.toRadians(45)))
                                .addDisplacementMarker(() -> { }).waitSeconds(0.5)
                                .addDisplacementMarker(55,() -> { })
                                .lineToSplineHeading(new Pose2d(-62,-55,Math.toRadians(180)))
                                .addDisplacementMarker(() -> { }).setReversed(true)
                                .waitSeconds(15).splineToConstantHeading(new Vector2d(10,-64),Math.toRadians(0))
                                .build());

        //warehouse bot
        RoadRunnerBotEntity warehouseBot = new DefaultBotBuilder(mm)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(57.635630404559784, 38.7, 4.5836622, Math.toRadians(60), 14.2)
                .setDimensions(13.2,16.603)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -63, Math.toRadians(-90)))
                                .addSpatialMarker(new Vector2d(40,-64),() -> { })
                                .addSpatialMarker(new Vector2d(5,-50),() -> { })
                                .lineToSplineHeading(new Pose2d(-2,-40, Math.toRadians(-240)))
                                .addDisplacementMarker(() -> { })
                                .setReversed(true).splineTo(new Vector2d(15, -62),Math.toRadians(-10))
                                .splineTo(new Vector2d(50,-64),Math.toRadians(0))
                                .back(5).forward(5)
                                .addDisplacementMarker(() -> { })
                                .addSpatialMarker(new Vector2d(0,-50),() -> { })
                                .splineTo(new Vector2d(-11.5,-42),Math.toRadians(90))
                                .addDisplacementMarker(() -> { })
                                .setReversed(true).addDisplacementMarker(() -> { })
                                .addSpatialMarker(new Vector2d(0,-50),() -> { })
                                .splineTo(new Vector2d(50,-64),Math.toRadians(0))
                                .back(5).forward(5)
                                .setReversed(false)
                                .splineTo(new Vector2d(-11.5,-42),Math.toRadians(90))
                                .addDisplacementMarker(() -> {})
                                .addSpatialMarker(new Vector2d(40,-64),() -> { })
                                .setReversed(true).addDisplacementMarker(() -> { })
                                .splineTo(new Vector2d(50,-64),Math.toRadians(0)).back(5).forward(5)
                                .setReversed(false)
                                .splineTo(new Vector2d(-11.5,-42),Math.toRadians(90))
                                .addDisplacementMarker(() -> { })
                                .addSpatialMarker(new Vector2d(40,-64),() -> { })
                                .setReversed(true).addDisplacementMarker(() -> { })
                                .splineTo(new Vector2d(50,-64),Math.toRadians(0))
                                .build());

        mm.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.9f)

                // Add both bot entities
                .addEntity(depotBot)
                .addEntity(warehouseBot)
                .start();
    }
}