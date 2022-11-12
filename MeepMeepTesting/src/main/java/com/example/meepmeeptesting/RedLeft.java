package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

//meep meep example create your own class
public class RedLeft {
    public static void main(String[] args) {
        //placeholder signal zone
        int signalZone = 1;
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = null;

        switch (signalZone){
            case 1:
                System.out.println("Going to 1");
                myBot = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .setDimensions(14, 10.5)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-35.5, -64.75, 1.57))
                                        /*
                                        .strafeRight(25)
                                        .forward(30)
                                        .strafeLeft(13)
                                        .forward(4)
                                        //use claw
                                        .back(4)
                                        .strafeRight(13)
                                        .back(30)
                                        .strafeLeft(50)
                                        .forward(30)
                                        .build()
                                         */

                                        .forward(17.5)
                                        .turn(Math.toRadians(-90))
                                        .forward(4)
                                        //use claw
                                        .back(4)
                                        .turn(Math.toRadians(90))
                                        .forward(3.5)
                                        .back(17)
                                        .turn(Math.toRadians(-90))
                                        .forward(24)
                                        .turn(Math.toRadians(90))
                                        .forward(48)
                                        .turn(Math.toRadians(90))
                                        .forward(48)
                                        .build()
                        );
                break;
            case 2:
                System.out.println("Going to 2");
                myBot = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .setDimensions(14, 10.5)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-35.5, -64.75, 1.57))

                                        /*
                                        .strafeRight(25)
                                        .forward(30)
                                        .strafeLeft(13)
                                        .forward(4)
                                        //use claw
                                        .back(4)
                                        .strafeRight(13)
                                        .forward(25)
                                        .strafeLeft(25)
                                        .build()
                                        */

                                        .forward(17.5)
                                        .turn(Math.toRadians(-90))
                                        .forward(4)
                                        //use claw
                                        .back(4)
                                        .turn(Math.toRadians(90))
                                        .forward(3.5)
                                        .back(17)
                                        .turn(Math.toRadians(-90))
                                        .forward(24)
                                        .turn(Math.toRadians(90))
                                        .forward(48)
                                        .turn(Math.toRadians(90))
                                        .forward(24)
                                        .build()
                        );
                break;
            case 3:
                System.out.println("Going to 3");
                myBot = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .setDimensions(14, 10.5)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-35.5, -64.75, 1.57))
                                        /*
                                        .strafeRight(25)
                                        .forward(30)
                                        .strafeLeft(13)
                                        .forward(4)
                                        //use claw
                                        .back(4)
                                        .strafeRight(13)
                                        //.forward(25)
                                        //.strafeLeft(50)
                                         */


                                        /*.strafeRight(12)
                                        .forward(10)
                                        //use claw
                                        .back(10)*/

                                        .forward(17.5)
                                        .turn(Math.toRadians(-90))
                                        .forward(4)
                                        //use claw
                                        .back(4)
                                        .turn(Math.toRadians(90))
                                        .forward(3.5)
                                        .back(17)
                                        .turn(Math.toRadians(-90))
                                        .forward(24)
                                        .turn(Math.toRadians(90))
                                        .forward(24)
                                        .build()
                        );

        }




        if(myBot != null){
            meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }

    }
}


