package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class RR_Freight_test {
    public static void main(String args[]) {
        MeepMeep mm = new MeepMeep(801)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15).setBotDimensions(13.2,16.603)
                .followTrajectorySequence(drive ->
                    drive.trajectorySequenceBuilder(new Pose2d(-31.2, -61.603, Math.toRadians(270)))
                        .addTemporalMarker(0,() -> {
                            /*
                            while(opModeisActive()) {mechanisms.maintainBalance}
                             */
                        })
                        .lineToSplineHeading(new Pose2d(-31,-24,0))
                        .addDisplacementMarker(() -> {
                            /* mechanisms.rotateArm(0.3, 600); */
                        }).waitSeconds(2).addDisplacementMarker(() -> {
                            /*
                            mechanisms.releaseServoMove(0.5);
                             */
                    }).waitSeconds(0.75)
                .build())
                .start();
    }
}
