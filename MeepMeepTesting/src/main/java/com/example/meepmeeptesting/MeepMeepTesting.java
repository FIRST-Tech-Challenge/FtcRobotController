package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        //3 Sample + 1 Specimen Auto
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-6, -60, 1.5708))
                .lineToY(-30)
                //Arm pos up to 0.5
                .lineToY(-24) //go past the bars
                //Arm Pos down to 0.7
                //Loosen Claw
                .lineToY(-40)  // drive back and hope the specimen attaches
                //Arm Pos to 0.5
                .strafeTo(new Vector2d(-50, -40))
                //Arm Pos to 1
                //Pick up Sample
                //Arm Pos to 0.5
                .turn(Math.toRadians(150)) //rotate to face the bucket
                .lineToY(-50)
                //Arm Pos to 0.7 or something
                //release Sample
                //Arm pos to 0.5
                .turn(Math.toRadians(-150)) // rotate to face second sample
                .strafeTo(new Vector2d(-60,-40)) // may have to adjust because claw is off center
                //Arm Pos to 1
                //grab sample
                .turn(Math.toRadians(175))
                .lineToY(-50)
                //Arm Pos to 0.7 or something
                // release claw
                //Arm Pos to 0.5
                .turn(Math.toRadians(185))
                .strafeTo(new Vector2d(-55, -25)) // x may too far to the left
                .turn(Math.toRadians(90))
                //lower Arm Pos to 1
                //Grab Sample
                .turn(Math.toRadians(75))
                .lineToY(-50)
                //lower Arm to 0.7
                //drop sample
                //will work on park later(LOL AS IF WE ARE GOING TO GREAT 3+1 working)

                .build());


//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-12, -60, 1.5708))
//                .lineToY(-36)
//                .turn(Math.toRadians(-90))
//                .lineToX(0)
//                .turn(Math.toRadians(90))
//                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}