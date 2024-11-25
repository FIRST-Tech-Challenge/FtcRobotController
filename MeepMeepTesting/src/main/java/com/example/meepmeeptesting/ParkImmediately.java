package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class ParkImmediately {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Create paths for each starting position
        RoadRunnerBotEntity redRight = new DefaultBotBuilder(meepMeep)
                .setDimensions(15, 15)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(
                        new Pose2d(36, -60, Math.toRadians(90)))
                        .strafeRight(24)
                        .build());

        RoadRunnerBotEntity redLeft = new DefaultBotBuilder(meepMeep)
                .setDimensions(15, 15)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(
                        new Pose2d(-36, -60, Math.toRadians(90)))
                        .strafeRight(80)
                        .build());

        RoadRunnerBotEntity blueRight = new DefaultBotBuilder(meepMeep)
                .setDimensions(15, 15)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(
                        new Pose2d(36, 60, Math.toRadians(-90)))
                        .strafeRight(80)
                        .build());

        RoadRunnerBotEntity blueLeft = new DefaultBotBuilder(meepMeep)
                .setDimensions(15, 15)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(
                        new Pose2d(-36, 60, Math.toRadians(-90)))
                        .strafeRight(24)
                        .build());

        redRight.setLooping(false);
        redLeft.setLooping(false);
        blueRight.setLooping(false);
        blueLeft.setLooping(false);
        
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueRight)
                .addEntity(redRight)
                .addEntity(blueLeft)
                .addEntity(redLeft)
                .start();
    }
}