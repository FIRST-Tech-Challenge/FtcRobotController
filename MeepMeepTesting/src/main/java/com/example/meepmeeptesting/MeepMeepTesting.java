package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity colorBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        RoadRunnerBotEntity yellowBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        //Starting Positions
        Pose2d startColor = new Pose2d(12,-60,Math.toRadians(90));
        Pose2d startYellow = new Pose2d(-12, -60, Math.toRadians(90));

        Vector2d basketDropOff = new Vector2d(-55, -65);

        yellowBot.runAction(yellowBot.getDrive().actionBuilder(startYellow)
                //Hang Preload
                .lineToY(-34)
                .waitSeconds(3) //Hang Specimen
                //Back up to avoid crashing
                .strafeToLinearHeading(new Vector2d(-12,-46),Math.toRadians(180))

                //Pick up new sample
                .strafeTo(new Vector2d(-38,-26))
                //Claw/Arm stuff
                .waitSeconds(3)//Pick up Spike mark

                //Drop off sample in basket
                //Raise lift and arm mid drive
                .strafeTo(basketDropOff)
                .waitSeconds(1)//Drop sample

                //Travel to next Spike Mark
                .strafeTo(new Vector2d(-48,-26))
                //Pick up Spike mark
                .waitSeconds(3)
                .strafeTo(basketDropOff)
                .waitSeconds(1)//Drop Sample

                //Travel to next Spike Mark
                .strafeTo(new Vector2d(-58,-26))
                //Pick up Spike mark
                .waitSeconds(3)
                .strafeTo(basketDropOff)
                .waitSeconds(1)//Drop Sample

                .build());


        //COLOR SIDE ROUTE
        colorBot.runAction(colorBot.getDrive().actionBuilder(startColor)

            //Hang Preload
                .lineToY(-34)
                .waitSeconds(3) //Hang Specimen
                //Back up to avoid crashing
                .strafeToLinearHeading(new Vector2d(12,-46),Math.toRadians(0))

            //Pick up new sample
                .strafeTo(new Vector2d(38,-26))
                //Claw/Arm stuff
                .waitSeconds(3)//Pick up Spike mark


            //Drop of sample for HP
                .strafeTo(new Vector2d(48,-60))
                .waitSeconds(1)//Drop Sample
                .strafeTo(new Vector2d(48,-44))
//                        .turn(Math.toRadians(-90))
            //Retrieve second sample
                .strafeTo(new Vector2d(48,-26))
                        .waitSeconds(1)//Pick up new Sample
                .turn(Math.toRadians(-90))

//                .waitSeconds(5)//Place sample in HP zone, wait for HP

            //Pick up specimen from HP
                .strafeTo(new Vector2d(48,-60))
                //Extend/prepare claw here
                .strafeTo(new Vector2d(60,-60))
                //Close claw
                .waitSeconds(1)

            //Drive and hang specimen
                .setReversed(true)
                                .strafeToLinearHeading(new Vector2d(11, -34),Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(11,-34,Math.toRadians(90 - 1e-6)),Math.toRadians(90))//Travel to hanging bar
                .waitSeconds(3)// Hang Specimen Again

            //Park in ascent zone

                .strafeToLinearHeading(new Vector2d(-36,-36),0)
                .strafeTo(new Vector2d(-36,-12))
                //Rise lift and arm to touch/rest on bar
                .strafeTo(new Vector2d(-24,-12))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(colorBot)
                .start();
    }
}