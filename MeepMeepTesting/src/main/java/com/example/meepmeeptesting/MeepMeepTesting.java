package com.example.meepmeeptesting;


import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static double dummyP = 3;

    public static double dropX = 30, dropY = 19.5, dropA = toRadians(315), dropET = toRadians(150);

    public static double pickupX1 = -46, pickupY1 = 10, pickupA1 = toRadians(180), pickupET1 = toRadians(180);
    public static double pickupX2 = 64.69, pickupY2 = 11, pickupA2 = toRadians(0), pickupET2 = toRadians(0);

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(600);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13.5,14.5)
                .setConstraints(50, 40, 4 * PI, 2 * PI, 11)
                .setConstraints(60, 60, toRadians(180), toRadians(180), 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(33, 63.25, Math.toRadians(90)))
                                        .setReversed(true)
                                        .lineToLinearHeading(new Pose2d(34, 48, toRadians(90)))
                                        .splineToSplineHeading(new Pose2d(34.5, 12, toRadians(315)), toRadians(270))
                                        .lineToLinearHeading(new Pose2d(dropX, dropY, toRadians(315)))
                                        .setReversed(false)
                                        .splineTo(new Vector2d(pickupX2,pickupY2), pickupET2)
                                        .setReversed(true)
//                                        .splineToSplineHeading(new Pose2d(dropX,dropY,dropA), dropET)
                                        .splineTo(new Vector2d(dropX,dropY), dropET)

//                                        .splineToSplineHeading(new Pose2d(dropX, dropY, dropA), dropET)
                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}