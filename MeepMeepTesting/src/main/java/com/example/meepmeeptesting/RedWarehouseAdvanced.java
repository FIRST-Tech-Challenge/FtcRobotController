package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class RedWarehouseAdvanced {
    public static void main(String args[]) {
        MeepMeep mm = new MeepMeep(800)
                // Set field image.
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(38.7, 38.7, Math.toRadians(180), Math.toRadians(180), 15).setBotDimensions(11,13.835)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -63, Math.toRadians(-90)))
                                .addTemporalMarker(6500,() -> { /* mechanisms.moveIntake(0.7); */ })
                                .addTemporalMarker(20000,() -> { /* mechanisms.moveIntake(0.7); */ })
                                .lineToSplineHeading(new Pose2d(-2,-40, Math.toRadians(-240)))
                                .addDisplacementMarker(() -> {
                                    /* timeout.reset();
                                    mechanisms.rotateArm(1400,0.5);
                                    while(mechanisms.armDC.getCurrentPosition() <= 1400 || timeout.milliseconds() <= 3000) {
                                        mechanisms.maintainBalance();
                                    } */
                                }).waitSeconds(0.75)
                                .addDisplacementMarker(() -> { /* mechanisms.releaseServoMove(0.6); */ })
                                .addDisplacementMarker(() -> { /*mechanisms.reset(); */ })
                                .setReversed(true).splineTo(new Vector2d(8, -60),Math.toRadians(-20)).splineTo(new Vector2d(50,-64),Math.toRadians(0)).setReversed(true)
                                //.lineToLinearHeading(new Pose2d(12,-64,Math.toRadians(180))).strafeTo(new Vector2d(50,-64))//.splineToSplineHeading(new Pose2d(4,-62,Math.toRadians(180)),Math.toRadians(90)).splineToConstantHeading(new Vector2d(50, -64), Math.toRadians(0))
                                .back(5).forward(5).back(5).forward(5)
                                /* .splineToConstantHeading(new Vector2d(36, -64),0) */
                                .addDisplacementMarker(() -> { /*mechanisms.reset(); */ }).splineTo(new Vector2d(-11.5,-41),Math.toRadians(90))
                                .setReversed(true).waitSeconds(1).splineTo(new Vector2d(50,-64),Math.toRadians(0))
                                //splineToLinearHeading(new Pose2d(0,-60,Math.toRadians(180)),Math.toRadians(0)).lineToSplineHeading(new Pose2d(50,-64,Math.toRadians(180)))//.splineToLinearHeading(new Pose2d(50,-64,Math.toRadians(180)),Math.toRadians(270))
                                //.splineToLinearHeading(new Pose2d(0,-62,Math.toRadians(180)),Math.toRadians(45)).splineToConstantHeading(new Vector2d(50,-64),Math.toRadians(180))
                                .build())
                .start();
    }
}