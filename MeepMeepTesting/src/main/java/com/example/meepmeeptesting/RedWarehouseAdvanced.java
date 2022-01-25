package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class RedWarehouseAdvanced {
    public static void main(String args[]) {
        MeepMeep mm = new MeepMeep(800)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15).setBotDimensions(11,13.835)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(6, -63, Math.toRadians(90)))
                                .addTemporalMarker(6500,() -> { /* mechanisms.moveIntake(0.7); */ })
                                .addTemporalMarker(20000,() -> { /* mechanisms.moveIntake(0.7); */ })
                                .lineToSplineHeading(new Pose2d(8,-24,0))
                                .addDisplacementMarker(() -> {
                                    /* timeout.reset();
                                    mechanisms.rotateArm(1400,0.5);
                                    while(mechanisms.armDC.getCurrentPosition() <= 1400 || timeout.milliseconds() <= 3000) {
                                        mechanisms.maintainBalance();
                                    } */
                                }).waitSeconds(1.5)
                                .addDisplacementMarker(() -> { /* mechanisms.releaseServoMove(0.6); */ }).waitSeconds(1)
                                .addDisplacementMarker(() -> { /*mechanisms.reset(); */ }).splineTo(new Vector2d(8, -55), Math.toRadians(270))
                                .splineTo(new Vector2d(36, -64),0).strafeTo(new Vector2d(47, -64)).waitSeconds(3)
                                .addDisplacementMarker(() -> { /*mechanisms.reset(); */ }).strafeTo(new Vector2d(15,-64))
                                .lineToSplineHeading(new Pose2d(-11.5,-41,Math.toRadians(-90))).waitSeconds(3)
                                .splineTo(new Vector2d(45,-64),0)
                                .build())
                .start();
    }
}