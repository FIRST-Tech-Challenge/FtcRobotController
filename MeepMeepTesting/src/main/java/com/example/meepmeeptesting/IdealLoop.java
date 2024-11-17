package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class IdealLoop {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity ideal = new DefaultBotBuilder(meepMeep)
                .setDimensions(16, 16)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(
                        new Pose2d(-36, 60, Math.toRadians(270)))
                        .splineToSplineHeading(new Pose2d(-14, 30, Math.toRadians(270)), Math.toRadians(270))
                        .waitSeconds(1)
                        .lineTo(new Vector2d(-14, 37))
                        .splineToSplineHeading(new Pose2d(-50, 55, Math.toRadians(90)), Math.toRadians(180))
                        .waitSeconds(1)
                        .splineTo(new Vector2d(-14, 30), Math.toRadians(-90))
                        .waitSeconds(1)
                        .lineTo(new Vector2d(37, 30))
                        .splineTo(new Vector2d(25, 10), Math.toRadians(270))
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setShowFPS(true)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(ideal)
                .start();
    }
}