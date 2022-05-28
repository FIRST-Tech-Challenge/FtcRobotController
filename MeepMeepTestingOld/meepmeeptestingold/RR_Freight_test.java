package com.example.meepmeeptestingold;

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
                    drive.trajectorySequenceBuilder(new Pose2d(50,-64,180)).splineTo(new Vector2d(-11.5,-41),Math.toRadians(90)).build())
                .start();
    }
}
