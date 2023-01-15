package com.example.meepmeeptesting;


import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class BlueRightAutoHighMeep {

    public static double dropX = -31.75, dropY = 2, dropA = toRadians(130), dropET = toRadians(310);

    public static double pickupX1 = -45.5, pickupY1 = 11.75, pickupA1 = toRadians(180), pickupET1 = toRadians(180);
    public static double pickupX2 = -63.2, pickupY2 = 11.75, pickupA2 = toRadians(180), pickupET2 = toRadians(180);

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(700);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13.5, 14.5)
                .setConstraints(80, 40, toRadians(180), toRadians(180), 14)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-29.6, 62.25, toRadians(60)))
                                        .setReversed(true).splineTo(new Vector2d(-36, 40), toRadians(260))
//                                        .splineTo(new Vector2d(-36, 27), toRadians(280))
                                        .splineTo(new Vector2d(-35, 16), toRadians(310))
                                        .addTemporalMarker(()->{})
                                        .splineToSplineHeading(new Pose2d(-29.2, 6.4, toRadians(140)), toRadians(310))
                                        .setReversed(false)
                                        .splineToSplineHeading(new Pose2d(-62.9, 12.5, toRadians(180)), toRadians(180))
                                        .setReversed(true)
//                                        .splineToSplineHeading(new Pose2d(-40,9.5,toRadians(140)),toRadians(330))
                                        .splineToConstantHeading(new Vector2d(dropX, dropY), toRadians(330))
                                        .setReversed(false)
//                                        .splineToSplineHeading(new Pose2d(-45.5, 10, toRadians(180)), toRadians(180))
                                        .splineToSplineHeading(new Pose2d(-63.2, 10.5, toRadians(180)), toRadians(180))
                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(dropX, dropY, dropA), dropET)
                                        .setReversed(false)
                                        .splineToSplineHeading(new Pose2d(-63.2, 10.5, toRadians(180)), toRadians(180))
                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(dropX, dropY, dropA), dropET)
                                        .setReversed(false)
                                        .splineToSplineHeading(new Pose2d(-63.2, 10.5, toRadians(180)), toRadians(180))
                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(dropX, dropY, dropA), dropET)
                                        .setReversed(false)
                                        .splineToSplineHeading(new Pose2d(-63.2, 10.5, toRadians(180)), toRadians(180))
                                        .setReversed(true)
                                        .splineToSplineHeading(new Pose2d(dropX, dropY, dropA), dropET)
                                        .setReversed(false)
                                        .setReversed(false)
//                                        .splineToConstantHeading(new Vector2d(-35,6), toRadians(50))
                                        .setReversed(false)
                                        .splineToLinearHeading(new Pose2d(dropX-4,dropY+7, toRadians(185)),toRadians(dropA))
                                        .setReversed(true)
                                        .splineToLinearHeading(new Pose2d(-10,12,toRadians(180)), toRadians(0))
//                                        .splineTo(new Vector2d(-10,12), toRadians(0))//
////                                        .splineToConstantHeading(new Vector2d(-10,10), toRadians(0))
//                                        .setReversed(true)
//                                        .addDisplacementMarker(10,() -> meepMeep.setDarkMode(true))
//                                        .addDisplacementMarker(() -> meepMeep.setDarkMode(false))
//                                        .UNSTABLE_addDisplacementMarkerOffset(10,()->{})
//                                        .splineTo(new Vector2d(-35.25, 47.5), toRadians(270))
//                                        .addTemporalMarker(()->{})
//                                        .UNSTABLE_addDisplacementMarkerOffset(10,()->{})
//                                        .splineTo(new Vector2d(-23.5, 35), toRadians(0))
//                                        .addTemporalMarker(()->{})
//                                        .splineToSplineHeading(new Pose2d(-11.75+5, 35.25+6, toRadians(225)), toRadians(45))
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}