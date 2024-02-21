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
    public static double yStack = -12;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = new Pose2d(new Vector2d(-37, -64.25), Math.toRadians(90));



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 48, 4.4, Math.toRadians(180), 15.5)


                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToSplineHeading(new Pose2d(new Vector2d(50, -12), Math.toRadians(0)))
                                .splineToSplineHeading(new Pose2d(39, -12, Math.toRadians(225+180)), Math.toRadians(225))

//                                //.splineToLinearHeading( new Vector2d(-36, -14), Math.toRadians(45))
//                                .lineToSplineHeading(new Pose2d(new Vector2d(-37, -30), Math.toRadians(90)))
//                                .splineToSplineHeading(new Pose2d(-36, -12, Math.toRadians(45)), Math.toRadians(45))
//                                .back (3)
//                                .turn(Math.toRadians(135))
//
//                                .splineToLinearHeading(new Pose2d(new Vector2d(xStack-8, yStack),Math.toRadians(180)),Math.toRadians(180))
//                                .lineTo(new Vector2d(-45, -12))
//                                .lineToLinearHeading(new Pose2d(-36,-11, Math.toRadians(45+180)))
//
////
////
////                                .splineToSplineHeading(new Pose2d(-32, -9, Math.toRadians(45+180)), Math.toRadians(45))
//
//                                .splineToLinearHeading(new Pose2d(new Vector2d(-45, -12),Math.toRadians(180)),Math.toRadians(180))
//                                .lineTo(new Vector2d(xStack-8, yStack))

//                                .lineToSplineHeading(new Pose2d(new Vector2d(-45, -12),Math.toRadians(180)))
//                                .splineToSplineHeading(new Pose2d(-32, -9, Math.toRadians(45+180)), Math.toRadians(45))

                                .lineToSplineHeading(new Pose2d(new Vector2d(-32.5, -38), Math.toRadians(90)))
                                .splineTo(new Vector2d(-33.5, -14.5 ), Math.toRadians(45))
//                                .forward (5)
//                                .turn(Math.toRadians(45))
//                                //middle
//                               // .lineToLinearHeading(new Pose2d(new Vector2d(-34, -24),Math.toRadians(270)))
//                                //left
//                              //  .lineToLinearHeading(new Pose2d(new Vector2d(-58, -12), Math.toRadians(270)))
//                                //right
//                                .lineToLinearHeading(new Pose2d( new Vector2d(-12, -12), Math.toRadians(270)))
//                                .lineToLinearHeading(new Pose2d( new Vector2d(-12, -24), Math.toRadians(270)))



                                .build()
                );

        RoadRunnerBotEntity newmasters = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(60), Math.toRadians(60), 15.00)
                .setDimensions(15, 17)

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(12, 58.5), Math.toRadians(270)))

                                .setTangent(Math.toRadians(-40))
                                .splineToLinearHeading(new Pose2d(36, 29, Math.toRadians(180)), Math.toRadians(-70))
                                .waitSeconds(1)


                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(46.5, 30, Math.toRadians(180)), Math.toRadians(0))
                                .waitSeconds(1)


                                .setTangent(Math.toRadians(120))
                                .splineToLinearHeading(new Pose2d(44, 58, Math.toRadians(-170)), Math.toRadians(80))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(newmasters)
                .start();
    }
}