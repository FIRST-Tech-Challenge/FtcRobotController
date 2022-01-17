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
                        drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(-41.5, -63),Math.toRadians(90)))
                                .lineToLinearHeading(hubPosition)
                                .lineToLinearHeading(new Pose2d(new Vector2d(-59.25, -59.25), Math.toRadians(90)))
                                .strafeRight(2)
                                .lineToLinearHeading(hubPosition)
                                .lineToLinearHeading(parkPosition)

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