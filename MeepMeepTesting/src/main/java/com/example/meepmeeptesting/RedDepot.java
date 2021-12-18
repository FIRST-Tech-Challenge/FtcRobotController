package com.example.meepmeeptesting;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

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
               .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15).setBotDimensions(12,15)
               .followTrajectorySequence(drive ->
                       drive.trajectorySequenceBuilder(new Pose2d(6, -63, Math.toRadians(90))).waitSeconds(5)
                               .build()).start();
   }
}
