package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class RedWarehouse {
    public static void main(String args[]) {
        MeepMeep mm = new MeepMeep(800)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(38.7, 38.7, 4.5836622, Math.toRadians(60), 14.2).setBotDimensions(13.2,16.603)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(6, -63, Math.toRadians(270)))
                                .strafeTo(new Vector2d(0,-45)).turn(Math.toRadians(-150)).waitSeconds(0.5)
                                .turn(Math.toRadians(60))
                                .strafeTo(new Vector2d(6, -64))
                                .strafeTo(new Vector2d(50,-64))
                                .build())
                .start();
    }
}