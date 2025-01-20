package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .build();


        Pose2d initialPose = new Pose2d(24, -61, Math.toRadians(90));

        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1Traj = myBot.getDrive().actionBuilder(initialPose)
                .waitSeconds(3)
                .strafeTo(new Vector2d(57, -37));

        Action tab1 = tab1Traj.build();

        Action trajectoryActionCloseOut = tab1Traj.fresh()
                .strafeTo(initialPose.component1()).build();

        // actions that need to happen on init; for instance, a claw tightening.
        myBot.runAction( new SequentialAction(
                tab1,
                trajectoryActionCloseOut));


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}