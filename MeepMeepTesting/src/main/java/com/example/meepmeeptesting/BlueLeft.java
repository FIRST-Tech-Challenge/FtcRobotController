package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueLeft {
    // from top left corner
        public static void main(String[] args) {
            int signalZone = 1;
            MeepMeep meepMeep = new MeepMeep(800);
            RoadRunnerBotEntity myBot = null;

            switch (signalZone){
                case 1:
                    // done
                    System.out.println("Going to 1");
                    myBot = new DefaultBotBuilder(meepMeep)
                            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                            .setDimensions(14, 10.5)
                            .followTrajectorySequence(drive ->
                                    drive.trajectorySequenceBuilder(new Pose2d(-35.5, 61, Math.toRadians(0)))
                                            //use claw
                                            .forward(12)
                                            .turn(Math.toRadians(-90))
                                            .forward(2)
                                            .turn(Math.toRadians(90))
                                            .forward(11)
                                            .turn(Math.toRadians(-90))
                                            .forward(45)
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
                                    drive.trajectorySequenceBuilder(new Pose2d(-35.5, 61, Math.toRadians(0)))
                                            .forward(12)
                                            .turn(Math.toRadians(-90))
                                            .forward(2)
                                            .turn(Math.toRadians(90))
                                            .forward(11)
                                            .turn(Math.toRadians(-90))
                                            .forward(25)
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
                                    drive.trajectorySequenceBuilder(new Pose2d(-35.5, 61, Math.toRadians(0)))
                                            .forward(12)
                                            .turn(Math.toRadians(-90))
                                            .forward(2)
                                            .turn(Math.toRadians(90))
                                            .forward(11)
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


