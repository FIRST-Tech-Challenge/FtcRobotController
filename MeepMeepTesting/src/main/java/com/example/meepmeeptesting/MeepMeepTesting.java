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
              .splineToSplineHeading(new Pose2d(-57, -55, Math.toRadians(225)), Math.toRadians(225)) //basket position
              .setTangent(Math.toRadians(225)).setReversed(true)
              .splineToSplineHeading(new Pose2d(-40, -36, Math.toRadians(270)), Math.toRadians(90))
              .splineToConstantHeading(new Vector2d(-40, -14), Math.toRadians(90))// beginning of loop back
              .splineToSplineHeading(new Pose2d(-52, -14, Math.toRadians(270)), Math.toRadians(180))
              .splineToConstantHeading(new Vector2d(-54, -26), Math.toRadians(270)) //first sample pickup
              .splineToSplineHeading(new Pose2d(-64, -46, Math.toRadians(225)), Math.toRadians(225)) //end of first lap

              .setTangent(Math.toRadians(225)).setReversed(true)
              .splineToSplineHeading(new Pose2d(-48, -36, Math.toRadians(270)), Math.toRadians(90))
              .splineToConstantHeading(new Vector2d(-48, -14), Math.toRadians(90))// beginning of loop back
              .splineToSplineHeading(new Pose2d(-60, -14, Math.toRadians(270)), Math.toRadians(180))
              .splineToConstantHeading(new Vector2d(-64, -26), Math.toRadians(270)) //second sample pickup
              .splineToSplineHeading(new Pose2d(-64, -45, Math.toRadians(225)), Math.toRadians(225)) //end of second lap

              .setTangent(Math.toRadians(225)).setReversed(true)
              .splineToSplineHeading(new Pose2d(-52, -36, Math.toRadians(270)), Math.toRadians(90))
              .splineToConstantHeading(new Vector2d(-52, -14), Math.toRadians(90))// beginning of loop back
              .splineToSplineHeading(new Pose2d(-68, -14, Math.toRadians(270)), Math.toRadians(180))
              .splineToConstantHeading(new Vector2d(-72, -22), Math.toRadians(270)) //third sample pickup
              .splineToConstantHeading(new Vector2d(-72, -45), Math.toRadians(270)) //end of third lap

              .setTangent(Math.toRadians(90))
              .splineToConstantHeading(new Vector2d(-24, -6), Math.toRadians(0))
              .build());




      meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_LIGHT)
              .setDarkMode(true)
              .setBackgroundAlpha(0.95f)
              .addEntity(myBotDaniel)
              .start();
   }
}