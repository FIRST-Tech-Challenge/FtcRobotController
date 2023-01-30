package com.example.meepmeeptesting;


import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class BlueRightAutoHighMeep {

    public static double dropX = -29.25, dropY = 4, dropA = toRadians(140);
    public static double pickupX1 = -45.5, pickupY1 = 11.75, pickupA1 = toRadians(180), pickupET1 = toRadians(180);
    public static double pickupX2 = -63.2, pickupY2 = 11.75, pickupA2 = toRadians(180), pickupET2 = toRadians(180);

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(700);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13.5, 14.5)
                .setConstraints(80, 40, toRadians(180), toRadians(180), 14)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-29.6, 62.25, toRadians(60)))
                                        .setReversed(true).splineTo(new Vector2d(-36, 40), toRadians(270))
//                .splineToSplineHeading(new Pose2d(-33, 14,toRadians(140)), toRadians(310))
                                        .splineToSplineHeading(new Pose2d(-27, 5, toRadians(120)), toRadians(300))
//                                        .addTemporalMarker(()->{})
//                                        .splineToSplineHeading(new Pose2d(-25, 6.4, toRadians(140)), toRadians(310))
                                        .setReversed(false)
                                        .splineToSplineHeading(new Pose2d(-62.9, 12.5, toRadians(180)), toRadians(180))
                                        .setReversed(true)
////                                        .splineToSplineHeading(new Pose2d(-40,9.5,toRadians(140)),toRadians(330))
                                        .splineToSplineHeading(new Pose2d(dropX, dropY,dropA), toRadians(320))
                                        .setReversed(false)
                                        .splineToSplineHeading(new Pose2d(-62.9, 12.5, toRadians(180)), toRadians(180))
                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(dropX, dropY,dropA), toRadians(320))


//                                        .splineToLinearHeading(new Pose2d(dropX-4,dropY+7, toRadians(90)),toRadians(90))
//                                        .lineToLinearHeading(new Pose2d(-10,12, toRadians(90)))

//                                        .lineToLinearHeading(new Pose2d(dropX-4,dropY+7, toRadians(90)))

                                        .setReversed(false)
                                        .splineToLinearHeading(new Pose2d(-60, 12, toRadians(90)), toRadians(180))


                                     .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}