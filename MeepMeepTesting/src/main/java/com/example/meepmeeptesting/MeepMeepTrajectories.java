package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

//meep meep example create your own class
public class MeepMeepTrajectories {
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
                                        .forward(22)
                                        //scan signal sleeve
                                        .back(22)
                                        .turn(Math.toRadians(-60))
                                        .forward(20)
                                        .turn(Math.toRadians(20))
                                        .forward(28)
                                        .turn(Math.toRadians(40))
                                        //use claw
                                        .turn(Math.toRadians(-40))
                                        .back(28)
                                        .turn(Math.toRadians(-20))
                                        .back(20)
                                        .turn(Math.toRadians(150))
                                        .forward(25)
                                        .turn(Math.toRadians(-90))
                                        .forward(30)
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
                                        .forward(22)
                                        //scan signal sleeve
                                        .back(22)
                                        .turn(Math.toRadians(-60))
                                        .forward(20)
                                        .turn(Math.toRadians(20))
                                        .forward(28)
                                        .turn(Math.toRadians(40))
                                        //use claw
                                        .back(3)
                                        .turn(Math.toRadians(40))
                                        .forward(35)
                                        .turn(Math.toRadians(50))
                                        .forward(15)
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
                                        .forward(22)
                                        //scan signal sleeve
                                        .back(22)
                                        .turn(Math.toRadians(-60))
                                        .forward(20)
                                        .turn(Math.toRadians(20))
                                        .forward(28)
                                        .turn(Math.toRadians(40))
                                        //use claw
                                        .back(3)
                                        .turn(Math.toRadians(90))
                                        .forward(10)
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


