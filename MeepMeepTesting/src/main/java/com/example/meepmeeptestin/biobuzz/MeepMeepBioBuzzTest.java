package com.example.meepmeeptestin.biobuzz;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepBioBuzzTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(14, 60.5, Math.toRadians(270)))

                // region block 1
                .waitSeconds(2) // Shoot
                .strafeToSplineHeading(new Vector2d(60.5,60.5), Math.toRadians(270))
                .setTangent(Math.toRadians(240))
                .strafeToLinearHeading(new Vector2d(55,55),Math.toRadians(220))
                .waitSeconds(1)
                // endregion

                //  region block 2 (off)
//                .setTangent(180)
//                .splineToLinearHeading(new Pose2d(14, 60.5, Math.toRadians(270)), Math.toRadians(180))
//                .waitSeconds(2) //shoot
//                .waitSeconds(1) //camera check
               //  endregion

                // region block 3

                .splineToLinearHeading(new Pose2d(40, -45, Math.toRadians(130)), Math.toRadians(250)) //move to loading side
                .waitSeconds(2) // shoot
                .waitSeconds(3) // intake
                .waitSeconds(1) // camera check

                // endregion

                // region block 4 (off)
//                .setTangent(Math.toRadians(330))
//                .splineToLinearHeading(new Pose2d(57, -20, Math.toRadians(180)), Math.toRadians(300))
                // endregion

                // region block 5 (off)
//                .splineToLinearHeading(new Pose2d(40, -45, Math.toRadians(130)), Math.toRadians(250)) //move to loading side
//                .waitSeconds(3) // intake human player
//                .waitSeconds(2) // shoot
//                .splineToLinearHeading(new Pose2d(57, -20, Math.toRadians(180)), Math.toRadians(300)) // park
                // endregion

                // region block 6 (off)

//                .setTangent(Math.toRadians(80))
//                .splineToLinearHeading(new Pose2d(14, 60.5, Math.toRadians(270)), Math.toRadians(160)) //move to garden side
//                .waitSeconds(2)// shoot
//                .setTangent(Math.toRadians(320))
//                .splineToLinearHeading(new Pose2d(57, -20, Math.toRadians(180)), Math.toRadians(300)) //park

                // endregion

                // region block 7
                .waitSeconds(2) // shoot
                .setTangent(Math.toRadians(50))
                .splineToLinearHeading(new Pose2d(57, -20, Math.toRadians(180)), Math.toRadians(50)) //park
                // endregion

                .build());

        Image img = null;
        try {
            // Provide the absolute path to your image file
            img = ImageIO.read(new File("MeepMeepTesting\\biobuzz.png"));
        } catch (IOException e) {
            e.printStackTrace();
        }

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
        }