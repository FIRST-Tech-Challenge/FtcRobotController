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

                                        .lineToLinearHeading(new Pose2d(-35.3,-12.1, Math.toRadians(45))) // Moves forward to the first tall pole and turns towards it
                                        // Deposit Preload Cone
                                        .turn(Math.toRadians(135)) // Turns towards cone stacks
                                        .forward(20) // Moves bot towards cones to grab one
                                        // Pickup a new cone from the stack
                                        .lineToLinearHeading(new Pose2d(-35.3,-12.1, Math.toRadians(225))) // Moves bot back to pole and faces it for backwards deposit
                                        // Deposit Pickup Cone
                                        .turn(Math.toRadians(45)) // Faces bot back towards middle parking position
                                        .forward(23.2) // Moves bot forwards towards middle parking position

                                        .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}