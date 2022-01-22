package com.example.meepmeeptesting;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

import java.util.Vector;

public class RedDepot {
   public static void main(String[] args) {
       MeepMeep mm = new MeepMeep(800)
               // Set field image
               .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
               // Set theme
               .setTheme(new ColorSchemeRedDark())
               // Background opacity from 0-1
               .setBackgroundAlpha(1f)
               // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
               .setConstraints(38.7, 30, 4.5836622, Math.toRadians(60), 14.2).setBotDimensions(13.2,16.603)
               .followTrajectorySequence(drive ->
                       drive.trajectorySequenceBuilder(new Pose2d(-34, -63, Math.toRadians(90)))
                               .strafeTo(new Vector2d(-32.5,-24)).turn(Math.toRadians(-90)).back(4).waitSeconds(2)
                               .strafeTo(new Vector2d(-62,-52)).waitSeconds(3).strafeTo(new Vector2d(-60,-36))
                               .build()).start();
   }
}
