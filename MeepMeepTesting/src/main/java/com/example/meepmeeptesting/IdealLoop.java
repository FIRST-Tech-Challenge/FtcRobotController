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
                        // starting location of blue right
                        new Pose2d(-36, 60, Math.toRadians(270)))
                        // moves in front of submersible to the right to pick up sample
                        .splineToSplineHeading(new Pose2d(-14, 30, Math.toRadians(270)), Math.toRadians(270))
                        .waitSeconds(1)
                        // moves back to net zone to score sample
                        .lineTo(new Vector2d(-14, 37))
                        // moves toward the wall
                        .splineToSplineHeading(new Pose2d(-37, 60, Math.toRadians(135)), Math.toRadians(90))
                        .waitSeconds(1)
                        // moves back in front of the submersible to the right to pick up sample
                        .lineTo(new Vector2d(-30,50))
                        .splineToSplineHeading(new Pose2d(-14, 30, Math.toRadians(-90)), Math.toRadians(-90))
                        .waitSeconds(1)
                        // moves to the left of the submersible
                        .lineTo(new Vector2d(40, 30))
                        // parks in observation zone to the left of the submersible
                        // robot faces away from rod so linear actuator makes contact and we score points
                        .lineToSplineHeading(new Pose2d(25, 10, Math.toRadians(0)))
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setShowFPS(true)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(ideal)
                .start();
    }
}