package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(300);

        final int BLUE = 0;
        final int RED = 1;

        final int TEAM1 = 0;
        final int TEAM2 = 1;


        int scannerOutput = 2;
        RoadRunnerBotEntity myBot;




                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                switch(scannerOutput){
                    case 1:
                        myBot = new DefaultBotBuilder(meepMeep)
                                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                                .followTrajectorySequence(drive ->
                                        drive.trajectorySequenceBuilder(new Pose2d(calculateStartingX(), calculateStartingY(), calculateStartingRotation()))
                                                .strafeRight(24)
                                                .forward(50.6)
                                                .strafeLeft(24)
                                                .build()
                                );
                        break;
                    case 2:
                        myBot = new DefaultBotBuilder(meepMeep)
                                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                                .followTrajectorySequence(drive ->
                                        drive.trajectorySequenceBuilder(new Pose2d(calculateStartingX(), calculateStartingY(), calculateStartingRotation()))
                                                .strafeLeft(24)
                                                .forward(26.6)
                                                .build()
                                );
                        break;
                    default:
                        myBot = new DefaultBotBuilder(meepMeep)
                                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                                .followTrajectorySequence(drive ->
                                        drive.trajectorySequenceBuilder(new Pose2d(calculateStartingX(), calculateStartingY(), calculateStartingRotation()))
                                                .strafeRight(24)
                                                .forward(26.6)
                                                .build()
                                );
                        break;
                }


        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static double calculateStartingX(){
        return 62.6;
    }
    public static double calculateStartingY(){
        return 36;
    }
    public static double calculateStartingRotation(){
        return Math.toRadians(180);
    }



}