package com.example.meepmeeptesting;


import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static double dropX = -31, dropY = 19 , dropA = toRadians(210), dropET = toRadians(30);

    public static double pickupX1 = -46, pickupY1 = 10, pickupA1 = toRadians(180), pickupET1 = toRadians(180);
    public static double pickupX2 = -63.2, pickupY2 = 11.5, pickupA2 = toRadians(180), pickupET2 = toRadians(180);

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(700);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13.5,14.5)
                .setConstraints(50, 40, 4 * PI, 2 * PI, 11)
                .setConstraints(60, 60, toRadians(180), toRadians(180), 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-29.6, 62.25, toRadians(90)))
                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(-34, 46, toRadians(90)), toRadians(270))
                                        .splineToSplineHeading(new Pose2d(-30.5, 29, toRadians(135)), toRadians(270))
                                        .setReversed(false)
                                        .splineToLinearHeading(new Pose2d(-34,11.5, toRadians(180)),toRadians(240))
                                        .splineToSplineHeading(new Pose2d(pickupX2, pickupY2, pickupA2), pickupET2)
//                                        .setReversed(false)
                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(dropX,dropY, dropA), dropET)
                                        .setReversed(false)
                                        .splineToSplineHeading(new Pose2d(-50, 11, toRadians(180)), toRadians(180))
                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(-10, 11, toRadians(180)), toRadians(180))
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}