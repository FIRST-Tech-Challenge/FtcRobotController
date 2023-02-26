package com.example.meepmeepv;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepmeepv {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(23, -70, 90))


                                .lineToLinearHeading(new Pose2d(23,-10, Math.toRadians(90)))
                                .waitSeconds(0.2)
                                .back(3)
                                .strafeLeft(23)
                                .forward(2.5)
                                .waitSeconds(0.5)
                                .back(3)
                                .lineToLinearHeading(new Pose2d(60,-13,Math.toRadians(0)))
                                .waitSeconds(0.2)
                                .back(2)
                                .lineToLinearHeading(new Pose2d(0,-10,Math.toRadians(90)))
                                .forward(2.5)
                                .waitSeconds(0.5)
                                .back(2)

                                .strafeLeft(16)


                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}