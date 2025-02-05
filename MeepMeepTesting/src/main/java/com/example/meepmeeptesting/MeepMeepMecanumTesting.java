package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepMecanumTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.PI, Math.PI, 1298.9390355971905)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(9.5, -61.25, Math.PI / 2 ))
                .splineToConstantHeading(new Vector2d(0,-34), Math.PI)

        // Alliance Push Samples
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(9.5, -61.25, -Math.PI / 2 ))
//                .lineToY(-34)
//                .setTangent(0)
//                .lineToX(36)
//                .setTangent(Math.PI / 2)
//                .splineToConstantHeading(new Vector2d(46, -9), 0)
//                .setTangent(-Math.PI / 2)
//                .lineToY(-50)
//                .setTangent(Math.PI / 2)
//                .splineToConstantHeading(new Vector2d(56, -11), 0)
//                .setTangent(-Math.PI / 2)
//                .lineToY(-50)
//                .setTangent(Math.PI / 2)
//                .splineToConstantHeading(new Vector2d(61, -13), 0)
//                .setTangent(-Math.PI / 2)
//                .lineToY(-50)
//                .setTangent(Math.PI / 2)
//                .splineToConstantHeading(new Vector2d(36, -61), -Math.PI / 2)
//                .setTangent(Math.PI / 2)
//                .splineToConstantHeading(new Vector2d(6.5, -34), Math.PI / 2)
//                .setTangent(-Math.PI / 2)
//                .splineToConstantHeading(new Vector2d(36, -61), -Math.PI / 2)
//                .setTangent(Math.PI / 2)
//                .splineToConstantHeading(new Vector2d(4.5, -34), Math.PI / 2)
//                .setTangent(-Math.PI / 2)
//                .splineToConstantHeading(new Vector2d(36, -61), -Math.PI / 2)
//                .setTangent(Math.PI / 2)
//                .splineToConstantHeading(new Vector2d(2.5, -34), Math.PI / 2)
//                .setTangent(-Math.PI / 2)
//                .splineToConstantHeading(new Vector2d(36, -61), -Math.PI / 2)
//                .setTangent(Math.PI / 2)
//                .splineToConstantHeading(new Vector2d(0.5, -34), Math.PI / 2)


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}