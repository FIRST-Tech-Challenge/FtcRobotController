package com.example.meepmeeptesting;


import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class BlueRightAutoHighMeep {

    public static double dropX = -32, dropY = 4.9, dropA = toRadians(140), dropET = toRadians(320);

    public static double pickupX1 = -45.5, pickupY1 = 10, pickupA1 = toRadians(180), pickupET1 = toRadians(180);
    public static double pickupX2 = -63.2, pickupY2 = 10.5, pickupA2 = toRadians(180), pickupET2 = toRadians(180);

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(700);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13.5, 14.5)
                .setConstraints(60, 60, 6.057, 6.057, 14)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-29.6, 62.25, toRadians(60)))
//                                        .setReversed(true).splineToSplineHeading(new Pose2d(-36, 40, toRadians(70)), toRadians(250))
//                                        .splineToSplineHeading(new Pose2d(-36, 26, toRadians(105)), toRadians(285))
//                                        .splineToSplineHeading(new Pose2d(-28,7.9, toRadians(120)), toRadians(290))
//                                        .setReversed(false)
//                                        .splineToSplineHeading(new Pose2d(pickupX1, pickupY1, pickupA1), pickupET1)
//                                        .splineToSplineHeading(new Pose2d(pickupX2, pickupY2, pickupA2), pickupET2)
//                                        .setReversed(true)
//                                        .splineToSplineHeading(new Pose2d(dropX,dropY, dropA), dropET)
//                                        .setReversed(false)
//                                        .splineToSplineHeading(new Pose2d(-45.5, 10, toRadians(180)), toRadians(180))
//                                        .splineToSplineHeading(new Pose2d(-63.2, 10.5, toRadians(180)), toRadians(180))
//                                        .setReversed(true)
//                                        .splineToSplineHeading(new Pose2d(dropX,dropY, dropA), dropET)
//                                        .setReversed(false)
//                                        .splineToSplineHeading(new Pose2d(-60,13, toRadians(180)), toRadians(180))
                                        .setReversed(true)
//                                        .addDisplacementMarker(10,() -> meepMeep.setDarkMode(true))
//                                        .addDisplacementMarker(() -> meepMeep.setDarkMode(false))
                                        .UNSTABLE_addDisplacementMarkerOffset(10,()->{})
                                        .splineTo(new Vector2d(-35.25, 47.5), toRadians(270))
                                        .UNSTABLE_addDisplacementMarkerOffset(10,()->{})
                                        .splineTo(new Vector2d(-23.5, 35), toRadians(0))
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