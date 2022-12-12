package com.example.meepmeeptesting;


import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static double dropX = -30, dropY =19.8, dropA = toRadians(220), dropET = toRadians(40);

    public static double pickupX1 = -46, pickupY1 = 10, pickupA1 = toRadians(180), pickupET1 = toRadians(180);
    public static double pickupX2 = -63.2, pickupY2 = 10.75, pickupA2 = toRadians(180), pickupET2 = toRadians(180);

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
                                        .splineToSplineHeading(new Pose2d(-32, 32, toRadians(135)), toRadians(270))
                                        .setReversed(false)
                                        .splineToLinearHeading(new Pose2d(-34,13, toRadians(180)),toRadians(240))
//                                        .setReversed(false)
//                                        .splineToSplineHeading(new Pose2d(-23.5,11.75, toRadians(180)),toRadians(180))
                                        .splineToSplineHeading(new Pose2d(pickupX2, pickupY2, pickupA2), pickupET2)
                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(dropX,dropY, dropA), dropET)
                                        .setReversed(false)
                                        .splineToSplineHeading(new Pose2d(-45.5, 11, toRadians(180)), toRadians(180))
                                        .splineToSplineHeading(new Pose2d(-63.2, 10.5, toRadians(180)), toRadians(180))
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}