package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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

      RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
              // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
              .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
              .build();

      RoadRunnerBotEntity myBotSam = new DefaultBotBuilder(meepMeep)
              // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
              .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
              .build();

      RoadRunnerBotEntity myBotDaniel = new DefaultBotBuilder(meepMeep)
              // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
              .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
              .build();


      myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36, -55, Math.toRadians(90)))
              //.waitSeconds(2)
              .lineToY(-34)
              .lineToY(-40)
              .strafeTo(new Vector2d(-54, -40))
              .setTangent(Math.toRadians(90))
              .lineToYSplineHeading(-10, Math.toRadians(0))
              .setTangent(Math.toRadians(0))
              .lineToXSplineHeading(20, Math.toRadians(0))
              .splineToConstantHeading(new Vector2d(50,-36),0)
              .waitSeconds(2)
              .setReversed(true)
              .splineToConstantHeading(new Vector2d(20,-10),Math.toRadians(180))
              .setTangent(Math.toRadians(180))
              .lineToX(-60)
              .waitSeconds(2)
              .setTangent(Math.toRadians(0))
              .lineToXSplineHeading(20, Math.toRadians(0))
              .splineToConstantHeading(new Vector2d(50,-36),0)
              .waitSeconds(2)
              .build());

      myBot2.runAction(myBot2.getDrive().actionBuilder(new Pose2d(-36, 60, Math.toRadians(270)))
              .lineToYSplineHeading(33, Math.toRadians(0))
              .waitSeconds(2)
              .setTangent(Math.toRadians(90))
              .lineToY(55)
              .setTangent(Math.toRadians(0))
              .lineToX(32)
              .strafeTo(new Vector2d(44.5, 30))
              .turn(Math.toRadians(180))
              .lineToX(47.5)
              .waitSeconds(3)
              .build());

      myBotSam.runAction(myBotSam.getDrive().actionBuilder(new Pose2d(-36, -55, Math.toRadians(270)))
              .lineToY(-34)
              .lineToY(-40)
              .splineTo(new Vector2d(-57, -35), Math.toRadians(180))
              .waitSeconds(3)
              .lineToX(-52)
              .splineToConstantHeading(new Vector2d(-35 ,-57), Math.toRadians(0))
              .lineToX(20)
              .setTangent(0)
              .splineToConstantHeading(new Vector2d(50 ,-35), Math.toRadians(0))
              .waitSeconds(3)
              .build());

      myBotDaniel.runAction(myBotDaniel.getDrive().actionBuilder(new Pose2d(0,-55,Math.toRadians(90)))
              .splineToSplineHeading(new Pose2d(-50, -50, Math.toRadians(225)), Math.toRadians(225))
              .setTangent(Math.toRadians(225))
              .lineToY(-56) //basket location
              .strafeToSplineHeading(new Vector2d(-35, -35), Math.toRadians(270))
              .setTangent(Math.toRadians(90))
              .lineToY(-0)
              .setTangent(Math.toRadians(180))
              .lineToX(-46)  //first sample pickup
              .setTangent(Math.toRadians(270))
              .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225)), Math.toRadians(225)) //end of 1st lap

              .setTangent(Math.toRadians(45))
              .splineToLinearHeading(new Pose2d(-45, -0, Math.toRadians(270)), Math.toRadians(90))
              .setTangent(Math.toRadians(180))
              .lineToX(-55)  //second sample pickup
              .setTangent(Math.toRadians(270))
//              .strafeToSplineHeading(new Vector2d(-50, -50),Math.toRadians(225)) //these three lines are alternate backup path
//              .setTangent(Math.toRadians(225))
//              .lineToY(-56) //end of 2nd lap
              .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225)), Math.toRadians(225)) //end of 2nd lap

              .setTangent(Math.toRadians(90))
              .splineToLinearHeading(new Pose2d(-54, -0, Math.toRadians(270)), Math.toRadians(90))
              .setTangent(Math.toRadians(180))
              .lineToX(-61)
              .setTangent(Math.toRadians(270))
              .lineToY(-50) //end of 3rd lap

              .setTangent(Math.toRadians(90))
              .splineToConstantHeading(new Vector2d(-24, -10), Math.toRadians(0))
              .build());




      meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_LIGHT)
              .setDarkMode(true)
              .setBackgroundAlpha(0.95f)
              .addEntity(myBotDaniel)
              .start();
   }
}