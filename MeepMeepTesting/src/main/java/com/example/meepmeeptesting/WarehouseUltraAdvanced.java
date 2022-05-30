package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

//attempt to create a really advanced warehouse program
public class WarehouseUltraAdvanced {
        public static void main(String[] args) {

         MeepMeep mm = new MeepMeep(800);
             RoadRunnerBotEntity bot = new DefaultBotBuilder(mm)
            .setColorScheme(new ColorSchemeRedDark())
            .setConstraints(57.635630404559784, 38.7, 4.5836622, Math.toRadians(60), 14.2)
            .setDimensions(13.2,16.603)
            .followTrajectorySequence(drive ->
                    drive.trajectorySequenceBuilder(new Pose2d(10, -63, Math.toRadians(90)))
                            //MOVING_TO_HUB
                            .splineToSplineHeading(new Pose2d(-2, -40,Math.toRadians(-240)),Math.toRadians(130))
                            .setReversed(true)
                            //MOVING_TO_WAREHOUSE #1
                            .splineToSplineHeading(new Pose2d(11,-62,Math.toRadians(180)),Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(50,-63),Math.toRadians(180))
                            //.splineTo(new Vector2d(50,-64),Math.toRadians(180))
                            .build());

            mm.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(bot)
            .start();
        }
}
