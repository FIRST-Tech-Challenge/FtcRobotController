package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepMyFirstPath {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(35, 35, Math.toRadians(60), Math.toRadians(60), 14.07)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(-35.3, -58.5), Math.toRadians(90)))

                                        .forward(53)
                                        .turn(Math.toRadians(-40))
                                        .forward(9)
                                        .back(7)
                                        .turn(Math.toRadians(40))
                                        .back(20)

                                        .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}