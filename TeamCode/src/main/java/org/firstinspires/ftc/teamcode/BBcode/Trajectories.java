package org.firstinspires.ftc.teamcode.BBcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Trajectories {
    public static Action Blue_Right_Auto(MecanumDrive drive){
        Pose2d initialPose = new Pose2d(-15, 63, Math.toRadians(-90));
        return drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(-9,42), Math.toRadians(-90))
                .waitSeconds(1)//hook preloaded specimen
                .strafeTo(new Vector2d(-12,42))
                .splineToSplineHeading(new Pose2d(-36,24, Math.toRadians(0)), Math.toRadians(-90))
                .splineTo(new Vector2d(-42,12), Math.toRadians(180))
                .strafeTo(new Vector2d(-45,12))
                .strafeTo(new Vector2d(-45,48))
                .strafeTo(new Vector2d(-45,24))
                .splineTo(new Vector2d(-57,12), Math.toRadians(180))
                .strafeTo(new Vector2d(-57,48))
                .strafeTo(new Vector2d(-57,42))
                .waitSeconds(1)//wait for human player to pick up samples
                .strafeTo(new Vector2d(-63,63))
                .build();
    };
}

