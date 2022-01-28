package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepRedCarousel {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d hubPosition = new Pose2d(new Vector2d(-29, -37), Math.toRadians(45));
        Pose2d parkPosition = new Pose2d(new Vector2d(-61, -34), Math.toRadians(0));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width

                .setConstraints(45, 60, Math.toRadians(60), Math.toRadians(60), 16.4)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(-35, -63),Math.toRadians(270)))
//                                .lineToLinearHeading(hubPosition)
//                                .lineToLinearHeading(new Pose2d(new Vector2d(-59.25, -59.25), Math.toRadians(90)))
//                                .strafeRight(2)
//                                .lineToLinearHeading(hubPosition)
//                                .lineToLinearHeading(parkPosition)
//
//                                .build()

                                .lineToSplineHeading(new Pose2d(new Vector2d(-12.5, -42), Math.toRadians(90)))
                                .back(13)
//                                .lineToLinearHeading(new Pose2d( Vector2d(-24, 52), Math.toRadians(315)))
                                .splineToLinearHeading(new Pose2d( new Vector2d (-60, -60), Math.toRadians(0)), Math.toRadians(180))
                                .lineTo(new Vector2d(-55, -55))
                                .lineToLinearHeading(new Pose2d(-55, -54, Math.toRadians(270)))
                                .lineTo(new Vector2d(-55, -63))
                                .lineTo(new Vector2d(-50, -55))
                                .splineToLinearHeading(new Pose2d(-12.5, -43.4, Math.toRadians(270)), Math.toRadians(270))
//                                .strafeRight(20)
                                .splineToLinearHeading(new Pose2d(new Vector2d(-22, -44), Math.toRadians(270)), Math.toRadians(180))
                                .splineToLinearHeading (new Pose2d(new Vector2d(-62, -35),Math.toRadians(270)), Math.toRadians(180))

                                .build()
                )
                ;

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}