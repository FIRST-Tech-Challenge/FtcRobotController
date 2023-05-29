package com.example.meepmeeptesting;


import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class BlueRightAutoHighMeep {

    public static double dropX = -30.5, dropY = 2.5, dropA = toRadians(140);
    public static double pickupX1 = -45.5, pickupY1 = 11.75, pickupA1 = toRadians(180), pickupET1 = toRadians(180);
    public static double pickupX2 = -63.2, pickupY2 = 11.75, pickupA2 = toRadians(180), pickupET2 = toRadians(180);

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(700);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13.5, 14.5)
                .setConstraints(80, 40, toRadians(180), toRadians(180), 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-dropX,dropY,Math.toRadians(40)))
//                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(dropX, dropY, Math.toRadians(130)), Math.toRadians(320))
//                                .setReversed(false)
//                                .splineTo(new Vector2d(-64.5, 11.75), toRadians(180))
                                .setReversed(false)
//                                .splineToSplineHeading(new Pose2d(-45, 11, toRadians(130)),toRadians(0))
                                .splineTo(new Vector2d(50,11.5),Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(55, 19.5,Math.toRadians(90)),Math.toRadians(90))
//                                .setReversed(false)
//                                .splineTo(new Vector2d(-64.5, 11.75), toRadians(180))

                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}