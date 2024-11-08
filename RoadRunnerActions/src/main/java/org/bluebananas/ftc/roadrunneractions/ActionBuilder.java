package org.bluebananas.ftc.roadrunneractions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.function.Function;

public class ActionBuilder {
    //generates path for BlueRightOption1
    public static Action BlueRightOption1(Function<Pose2d, TrajectoryActionBuilder> builderFunction)
    {
        Pose2d startPose= new Pose2d(-15, 63, Math.toRadians(-90));
        TrajectoryActionBuilder builder = builderFunction.apply(startPose);
        return builder
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
                .waitSeconds(1)//wait for human player
                .strafeTo(new Vector2d(-63,63))
                .build();
    }
    //generates path for BlueLeft
    public static Action BlueLeft(Function<Pose2d, TrajectoryActionBuilder> builderFunction)
    {
        Pose2d startPose = new Pose2d(33, 63, Math.toRadians(-90));
        TrajectoryActionBuilder builder = builderFunction.apply(startPose);
        return builder
                .splineTo(new Vector2d(48,48), Math.toRadians(45))
                .waitSeconds(1)//deposit sample
                .turnTo(Math.toRadians(-90))
                .waitSeconds(1)//pick up sample
                .turnTo(Math.toRadians(45))
                .waitSeconds(1)//deposit sample
                .strafeToLinearHeading(new Vector2d(58,48), Math.toRadians(-90))
                .waitSeconds(1)//pick up sample
                .strafeToLinearHeading(new Vector2d(48,48), Math.toRadians(45))
                .waitSeconds(10)//deposit sample and wait for other alliance to park
                .splineTo(new Vector2d(24,60), Math.toRadians(180))
                .strafeTo(new Vector2d(-36,60))
                .build();
    }
    public static Action RedRightOption2(Function<Pose2d, TrajectoryActionBuilder> builderFunction) {
        Pose2d startPose = new Pose2d(10, -63, Math.toRadians(90.0));
        TrajectoryActionBuilder builder = builderFunction.apply(startPose);
        return builder
                .strafeTo(new Vector2d(10.0, -45.0))
                .waitSeconds(1)
                .strafeTo(new Vector2d(25, -45))
                .splineTo(new Vector2d(45, -10), Math.toRadians(0))
//                .strafeTo(new Vector2d(28, -10))
//                .strafeTo(new Vector2d(46, -10))
                .strafeTo(new Vector2d(45, -55))
                .strafeToLinearHeading(new Vector2d(45, -30), Math.toRadians(270.0))
                .waitSeconds(1)
                .strafeTo(new Vector2d(45, -45))
                .waitSeconds(1)
                .splineTo(new Vector2d(10, -45), Math.toRadians(90))
//                .splineTo(new Vector2d(46, -30), Math.toRadians(180))
                .build();
    }

    public static Action RedLeft(Function<Pose2d, TrajectoryActionBuilder> builderFunction) {
        Pose2d startPose = new Pose2d(0, 0, 0);
        TrajectoryActionBuilder builder = builderFunction.apply(startPose);
        return builder
                .build();
    }
}