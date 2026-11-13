package com.example.meepmeeptestin.biobuzz;

import static com.example.meepmeeptestin.Constants.GARDEN_BEGIN_POSE;
import static com.example.meepmeeptestin.Constants.GARDEN_CAMERA_CHECK_HEADING;
import static com.example.meepmeeptestin.Constants.GARDEN_CAMERA_CHECK_VECTOR;
import static com.example.meepmeeptestin.Constants.GARDEN_SHOOT_POSE;
import static com.example.meepmeeptestin.Constants.INTAKE_GARDEN_HEADING;
import static com.example.meepmeeptestin.Constants.INTAKE_GARDEN_VECTOR;
import static com.example.meepmeeptestin.Constants.LOADING_SHOOT_POSE;
import static com.example.meepmeeptestin.Constants.SLOW_VEL;
import static com.example.meepmeeptestin.Constants.TO_BOTTOM_LEFT;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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

        myBot.runAction(myBot.getDrive().actionBuilder(GARDEN_BEGIN_POSE)

                // region block 1
                .waitSeconds(2) // Shoot
                .strafeToSplineHeading(INTAKE_GARDEN_VECTOR, INTAKE_GARDEN_HEADING, SLOW_VEL) // Intake
                .setTangent(TO_BOTTOM_LEFT)
                .strafeToLinearHeading(GARDEN_CAMERA_CHECK_VECTOR,GARDEN_CAMERA_CHECK_HEADING)
                .waitSeconds(1)
                // endregion

                //  region block 2 (off)
//                .setTangent(180)
//                .splineToLinearHeading(GARDEN_SHOOT_POSE, Math.toRadians(180))
//                .waitSeconds(2) //shoot
//                .waitSeconds(1) //camera check
               //  endregion

                // region block 3

                .splineToLinearHeading(LOADING_SHOOT_POSE, TO_BOTTOM_LEFT) //move to loading side
                .waitSeconds(2) // shoot
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
//                .splineToLinearHeading(new Pose2d(57, -20, Math.toRadians(180)), Math.toRadians(300)) // park

                // endregion
                // region block 6.1
                .waitSeconds(1) // shoot
                .setTangent(Math.toRadians(50))
                .splineToLinearHeading(new Pose2d(60.0, -34, Math.toRadians(90)), Math.toRadians(50), new TranslationalVelConstraint(15.0)) // intake
                .waitSeconds(1) // intake

                .splineToLinearHeading(new Pose2d(40, -45, Math.toRadians(130)), Math.toRadians(220)) // shoot

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