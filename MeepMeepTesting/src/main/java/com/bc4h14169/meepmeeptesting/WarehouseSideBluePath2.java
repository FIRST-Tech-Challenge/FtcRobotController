package com.bc4h14169.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class WarehouseSideBluePath2 {
    public static void main(String[] args){

        /*
        Steps for Autonomous:
        1) Move to the right
        2) Move forward to the Shipping Hub
        3) Move back and then strafe across the wall
        4) Park in warehouse second square in
        */


        MeepMeep meepMeep = new MeepMeep(800);

        //Setup BotEntity's dimensions and constraints with our robot's parameters
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(16.5, 13.25)
                //Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(41.065033847887705,41.065033847887705, Math.toRadians(100), Math.toRadians(180), 13.2435)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(12,60, Math.toRadians(270)))
                            .strafeTo(new Vector2d(-12, 42))
                            .strafeTo(new Vector2d(-12, 60))
                            .strafeTo(new Vector2d(44, 60))
                            .strafeTo(new Vector2d(44, 40))
                      .build()
                    );
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start();
    }
}
