package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class spline {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(23.6, 20, Math.toRadians(180), Math.toRadians(180), 11)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12,-66,Math.PI/2))
                                .addDisplacementMarker(() -> {
//                                    n.goSpecimen(2);
                                })
                                .forward(18)
                                .addDisplacementMarker(() -> {
//                                    n.goSpecimenDown(1);
//                                    while (!n.isAtTargetHeight()) {
//                                        if (n.isAtTargetHeight()) {
//                                            n.release();
//                                        }
//                                    }
                                })
                                .strafeRight(24)
                                .forward(42)
                                .strafeRight(5)
                                .turn(Math.toRadians(-90))
                                .strafeRight(48)
                                .strafeLeft(48)
                                .forward(12)
                                .strafeRight(48)
                                .strafeLeft(48)
                                .forward(10)
                                .strafeRight(48)
                                .build());

        myBot.setDimensions(17,17);
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}