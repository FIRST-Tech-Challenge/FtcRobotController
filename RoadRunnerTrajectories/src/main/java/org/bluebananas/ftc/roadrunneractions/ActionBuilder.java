package org.bluebananas.ftc.roadrunneractions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;

import org.bluebananas.ftc.roadrunneractions.TrajectoryActionBuilders.RedBasketPose;

import java.util.function.Function;


public class ActionBuilder {

    //Path for RedBasketPose
    public static Action RedBasket(Function<Pose2d, TrajectoryActionBuilder> builderFunction)
    {

        TrajectoryActionBuilder builder = builderFunction.apply(RedBasketPose.init);
        return builder

                .strafeToLinearHeading(RedBasketPose.drop.position, RedBasketPose.drop.heading)
                .strafeToLinearHeading(RedBasketPose.inner_sample.position, RedBasketPose.inner_sample.heading)
                .setReversed(true)
                .strafeToLinearHeading(RedBasketPose.drop.position, RedBasketPose.drop.heading)
                .strafeToLinearHeading(RedBasketPose.middle_sample.position, RedBasketPose.middle_sample.heading)
                .setReversed(true)
                .strafeToLinearHeading(RedBasketPose.drop.position, RedBasketPose.drop.heading)
                .strafeToLinearHeading(RedBasketPose.outer_sample.position, RedBasketPose.outer_sample.heading)
                .setReversed(true)
                .strafeToLinearHeading(RedBasketPose.drop.position, RedBasketPose.drop.heading)
                .strafeToLinearHeading(RedBasketPose.submersible_park.position, RedBasketPose.submersible_park.heading)
                .build();
    }

    //Path for RedSpecimen
    public static Action RedSpecimen(Function<Pose2d, TrajectoryActionBuilder> builderFunction)
    {
        Pose2d startPose = new Pose2d(9, -64, Math.toRadians(90));
        TrajectoryActionBuilder builder = builderFunction.apply(startPose);
        return builder
                .strafeToLinearHeading(new Vector2d(0,-48), Math.toRadians(90))
                .waitSeconds(1.5)//raise mechanisms to clip

                .strafeToLinearHeading(new Vector2d(0,-36),Math.toRadians(90), new TranslationalVelConstraint(10))
                .waitSeconds(1.5)//lower mechanisms from clipping

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(38,-26, Math.toRadians(90) ), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(46,-12, Math.toRadians(0)), Math.toRadians(0))//apex of inner sample
                .strafeToLinearHeading(new Vector2d(46,-55),Math.toRadians(0))//inner sample to observation
                .waitSeconds(0.2)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(2, -48), Math.toRadians(90))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(2,-36),Math.toRadians(90), new TranslationalVelConstraint(10))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(38,-58), Math.toRadians(0))
                .waitSeconds(0.2)
                .strafeToLinearHeading(new Vector2d(4,-48),Math.toRadians(90))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(4,-36),Math.toRadians(90), new TranslationalVelConstraint(10))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(60,-60),Math.toRadians(90))
                .build();

    }
}