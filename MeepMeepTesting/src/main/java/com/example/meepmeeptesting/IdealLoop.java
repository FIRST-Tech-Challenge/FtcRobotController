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

                RoadRunnerBotEntity idealBlueRight = new DefaultBotBuilder(meepMeep)
                                .setDimensions(16, 16)
                                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                                .setColorScheme(new ColorSchemeBlueDark())
                                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(
                                                // starting location of blue right
                                                new Pose2d(-36, 60, Math.toRadians(270)))
                                                // moves in front of submersible to the right to pick up sample
                                                .splineToSplineHeading(new Pose2d(-14, 30, Math.toRadians(270)),
                                                                Math.toRadians(270))
                                                .waitSeconds(1)
                                                // moves back to net zone to score sample
                                                .lineTo(new Vector2d(-14, 37))
                                                // moves toward the wall
                                                .splineToSplineHeading(new Pose2d(-37, 60, Math.toRadians(135)),
                                                                Math.toRadians(90))
                                                .waitSeconds(1)
                                                // moves back in front of the submersible to the right to pick up sample
                                                .lineTo(new Vector2d(-30, 50))
                                                .splineToSplineHeading(new Pose2d(-14, 30, Math.toRadians(-90)),
                                                                Math.toRadians(-90))
                                                .waitSeconds(1)
                                                // moves to the left of the submersible
                                                .lineTo(new Vector2d(40, 30))
                                                // parks in observation zone to the left of the submersible
                                                // robot faces away from rod so linear actuator makes contact and we
                                                // score
                                                // points
                                                .lineToSplineHeading(new Pose2d(25, 10, Math.toRadians(180)))
                                                .build());

                // New configurations for other robots based on idealBlueRight
                RoadRunnerBotEntity idealBlueLeft = new DefaultBotBuilder(meepMeep)
                                .setDimensions(16, 16)
                                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                                .setColorScheme(new ColorSchemeBlueDark())
                                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(
                                                new Pose2d(36, 60, Math.toRadians(270)))
                                                .splineToSplineHeading(new Pose2d(14, 30, Math.toRadians(270)),
                                                                Math.toRadians(270))
                                                .waitSeconds(1)
                                                .lineTo(new Vector2d(-14, 37))
                                                .splineToSplineHeading(new Pose2d(-37, 60, Math.toRadians(135)),
                                                                Math.toRadians(90))
                                                .waitSeconds(1)
                                                .lineTo(new Vector2d(-30, 50))
                                                .splineToSplineHeading(new Pose2d(14, 30, Math.toRadians(-90)),
                                                                Math.toRadians(-90))
                                                .waitSeconds(1)
                                                .lineTo(new Vector2d(40, 30))
                                                .lineToSplineHeading(new Pose2d(23.4, -11.5, Math.toRadians(180)))
                                                .build());

                RoadRunnerBotEntity idealRedRight = new DefaultBotBuilder(meepMeep)
                                .setDimensions(16, 16)
                                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                                .setColorScheme(new ColorSchemeRedDark())
                                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(
                                                // starting location of blue left
                                                new Pose2d(36, -60, Math.toRadians(90)))
                                                // pick up sample from daouda
                                                .splineToSplineHeading(new Pose2d(14, -30, Math.toRadians(90)),
                                                                Math.toRadians(90))
                                                .waitSeconds(1)
                                                .lineTo(new Vector2d(14, -37))
                                                .splineToSplineHeading(new Pose2d(36, -60, Math.toRadians(-45)),
                                                                Math.toRadians(-45))
                                                .waitSeconds(1)
                                                .splineToSplineHeading(new Pose2d(14, -30, Math.toRadians(90)),
                                                                Math.toRadians(90))
                                                .waitSeconds(1)
                                                .lineTo(new Vector2d(-33.8, -30.2))
                                                .lineToSplineHeading(new Pose2d(-23.4, 12.1, Math.toRadians(0)))
                                                .build());

                RoadRunnerBotEntity idealRedLeft = new DefaultBotBuilder(meepMeep)
                                .setDimensions(16, 16)
                                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                                .setColorScheme(new ColorSchemeRedDark())
                                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(
                                                // starting location of red left
                                                new Pose2d(-36, -60, Math.toRadians(90)))
                                                .splineToSplineHeading(new Pose2d(-8.3, -29.8, Math.toRadians(90)),
                                                                Math.toRadians(90))
                                                .waitSeconds(1)
                                                .lineTo(new Vector2d(-9.5, -38.3))
                                                .splineToSplineHeading(new Pose2d(36.1, -60.2, Math.toRadians(-45)),
                                                                Math.toRadians(-45))
                                                .waitSeconds(1)
                                                .splineToSplineHeading(new Pose2d(-8.3, -29.8, Math.toRadians(90)),
                                                                Math.toRadians(90))
                                                .waitSeconds(1)
                                                .lineTo(new Vector2d(-33.8, -30.2))
                                                .lineToSplineHeading(new Pose2d(-23.3, -10.3, Math.toRadians(0)))
                                                .build());

                meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                                .setShowFPS(true)
                                .setDarkMode(true)
                                .setBackgroundAlpha(0.95f)
                                .addEntity(idealBlueRight)
                                .addEntity(idealRedRight)
                                .addEntity(idealBlueLeft)
                                .addEntity(idealRedLeft)
                                .start();
        }
}