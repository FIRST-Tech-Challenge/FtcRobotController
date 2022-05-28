package com.example.meepmeeptestingold;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

import java.util.Vector;

public class RedDepotBasic {
   public static void main(String[] args) {
       MeepMeep mm = new MeepMeep(800)
               // Set field image
               .setBackground(MeepMeep.Background.FIELD_FREIGHT_FRENZY)
               // Set theme
               .setTheme(new ColorSchemeRedDark())
               // Background opacity from 0-1
               .setBackgroundAlpha(1f)
               // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
               .setConstraints(57.635630404559784, 38.7, 4.5836622, Math.toRadians(60), 14.2).setBotDimensions(13.2,16.603)
               .followTrajectorySequence(drive ->
                       drive.trajectorySequenceBuilder(new Pose2d(-34, -63, Math.toRadians(-90)))
                               .addSpatialMarker(new Vector2d(-25,-39),() -> {
                                   /*
                                      mechanisms.rotateArm(Utility_Constants.THIRD_LEVEL_POS,0.65);
                                    */
                               })
                               .lineToSplineHeading(new Pose2d(-22,-40,Math.toRadians(45))).addDisplacementMarker(() -> {
                                   /*
                                      mechanisms.releaseServoMove(0.3);
                                    */
                               }).waitSeconds(0.5)//.setReversed(true)
                               .addDisplacementMarker(55,() -> {
                                   /*
                                      mechanisms.reset();
                                    */
                               })
                               .lineToSplineHeading(new Pose2d(-62,-55,Math.toRadians(180))).addDisplacementMarker(() -> {
                                /*
                                   mechanisms.rotateCarousel(0.6);
                                 */
                                }).setReversed(true)
                               .waitSeconds(1.0).splineToConstantHeading(new Vector2d(10,-64),Math.toRadians(0))

                               //.splineToConstantHeading(new Vector2d(50,-64),Math.toRadians(0))
                               .build()).start();
   }
}
