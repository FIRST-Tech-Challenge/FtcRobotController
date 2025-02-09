package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedPath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .build();




        Pose2d initialPose = new Pose2d(-36, -61, Math.toRadians(90));

        Vector2d basketPos = new Vector2d(-55,-55);


        TrajectoryActionBuilder builder1 = myBot.getDrive().actionBuilder(initialPose)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(-48,-42))
                .waitSeconds(0.5)
                .strafeToSplineHeading(basketPos,Math.toRadians(45))
                .waitSeconds(0.5)
                .strafeToSplineHeading(new Vector2d(-58,-42),Math.toRadians(90))
                .waitSeconds(0.5)
                .strafeToSplineHeading(basketPos,Math.toRadians(45));





        Action act1 = builder1.build();
        myBot.runAction(new SequentialAction(
                act1
        ));


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

//        TrajectoryActionBuilder tab1Traj = myBot.getDrive().actionBuilder(initialPose)
//                .waitSeconds(3)
//                .strafeTo(new Vector2d(57, -37));
//        Action tab1 = tab1Traj.build();
//        Action trajectoryActionCloseOut = tab1Traj.fresh()
//                .strafeTo(initialPose.component1()).build();
//
//        // actions that need to happen on init; for instance, a claw tightening.
//        myBot.runAction( new SequentialAction(
//                tab1,
//                trajectoryActionCloseOut));
    }
}