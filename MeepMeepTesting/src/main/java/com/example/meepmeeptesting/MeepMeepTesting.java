package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static  double xCenterJunction = 5;
    public static double yCenterJunction =-33;

    public static double xIntermediateStack =20;
    public static double yIntermediateStack = -12;
    public static double angleIntermediateStack = 270;
    public static double xStack =59;
    public static double yStack = -14;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 60, Math.toRadians(60), Math.toRadians(60), 16.4)


                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(0, 0), Math.toRadians(180)))

                                .splineToLinearHeading(new Pose2d(new Vector2d(-53, 0),Math.toRadians(180)),Math.toRadians(180))
                                //.turn(Math.toRadians(45))
                               // .lineTo(new Vector2d(31, -8))
                                //.splineToLinearHeading(new Pose2d(new Vector2d(53, -13),0),0)
//                                .splineToConstantHeading(new Vector2d(12,-61),Math.toRadians(90))
//                                .splineTo(new Vector2d(xCenterJunction, yCenterJunction),Math.toRadians(135))
//                                .back(3)
//                                .splineToLinearHeading(new Pose2d(new Vector2d(xIntermediateStack, yIntermediateStack), 0), 0)
//
//                                .splineTo(new Vector2d(xStack, yStack), 0)
//                               // .lineToConstantHeading(new Vector2d(49, -9))
//                                .splineToLinearHeading(new Pose2d(new Vector2d(30.5,-9.5), -45),Math.toRadians(135))
                              //  .splineToConstantHeading(new Vector2d(10,-24), Math.toRadians(135))
//                                .splineTo(new Vector2d(xIntermediateStack,yIntermediateStack), Math.toRadians(angleIntermediateStack))
//                                .splineTo(new Vector2d(xStack, yStack), Math.toRadians(180))
//                                .lineToLinearHeading(new Pose2d(new Vector2d(59,-14),Math.toRadians(0)))
//                                .back(8)
//                                .splineToLinearHeading(new Pose2d(new Vector2d(31,-11),Math.toRadians(315)),Math.toRadians(155))
//                                .lineToLinearHeading(new Pose2d(new Vector2d(59,-14),Math.toRadians(0)))
//                                .back(8)
//                                .splineToLinearHeading(new Pose2d(new Vector2d(31,-11),Math.toRadians(315)),Math.toRadians(155))
//                                .lineToLinearHeading(new Pose2d(new Vector2d(59,-14),Math.toRadians(0)))
//                                .back(8)
//                                .splineToLinearHeading(new Pose2d(new Vector2d(31,-11),Math.toRadians(315)),Math.toRadians(155))

                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}