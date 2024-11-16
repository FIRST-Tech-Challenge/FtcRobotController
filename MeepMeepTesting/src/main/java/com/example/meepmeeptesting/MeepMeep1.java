package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep1 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Pose2d beginPose = new Pose2d(13, -61.5, Math.toRadians(270));

        TrajectoryActionBuilder moveAwayFromBarrier = myBot.getDrive().actionBuilder(beginPose)
                .strafeTo(new Vector2d(13, -50))
                .waitSeconds(0.001);
        TrajectoryActionBuilder moveIntoSpec1Position = moveAwayFromBarrier.fresh()
                .strafeTo(new Vector2d(0, -28))
                .waitSeconds(0.001);
        TrajectoryActionBuilder driveBack = moveIntoSpec1Position.fresh()
                .strafeTo(new Vector2d(0, -35))
                .waitSeconds(0.001);
        TrajectoryActionBuilder pushSampleGrabSpec = driveBack.fresh()
                .strafeTo(new Vector2d(45, -35))
                .strafeTo(new Vector2d(45, -10))
                .splineTo(new Vector2d(55, -10), Math.toRadians(270))
                .strafeTo(new Vector2d(55, -51))
                .strafeTo(new Vector2d(55, -57))
                .strafeTo(new Vector2d(55, -45))
                .waitSeconds(3)
                .strafeTo(new Vector2d(55, -59))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goToSubSecondSpec = pushSampleGrabSpec.fresh()
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(55, -45))
                .turn(Math.toRadians(180))
                .strafeTo(new Vector2d(4,-45))
                .strafeTo(new Vector2d(4, -27))
                .waitSeconds(0.001);
        TrajectoryActionBuilder goBackAndPark = goToSubSecondSpec.fresh()
                .waitSeconds(1)
                .strafeTo(new Vector2d(4, -45))
                .splineTo(new Vector2d(47, -47), Math.toRadians(90))
                .strafeTo(new Vector2d(47, -58))
                .waitSeconds(0.001);

        Action moveAwayFromBarrierAction = moveAwayFromBarrier.build();
        Action moveIntoSpec1PositionAction = moveIntoSpec1Position.build();
        Action driveBackAction = driveBack.build();
        Action pushSampleGrabSpecAction = pushSampleGrabSpec.build();
        Action goToSubSecondSpecAction = goToSubSecondSpec.build();
        Action goBackAndParkAction = goBackAndPark.build();

        Action autoSequence = new SequentialAction(
                moveAwayFromBarrierAction, // Move away from the barrier
                moveIntoSpec1PositionAction, // Move into position to place the first spec
                driveBackAction, //drive back to put slides fully down
                pushSampleGrabSpecAction, //pushes sample into player person zone, then grabs spec
                goToSubSecondSpecAction, //go to sub to put on second spec
                goBackAndParkAction //park in player person zone
        );

        myBot.runAction(autoSequence);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}