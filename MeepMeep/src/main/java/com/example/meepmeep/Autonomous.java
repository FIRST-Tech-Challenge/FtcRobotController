package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.AddTrajectoryCallback;
import com.noahbres.meepmeep.roadrunner.AddTrajectorySequenceCallback;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Vector;

public class Autonomous {
    enum Position {
        START_RED_UP(new Pose2d(12, -60, Math.toRadians(90))),
        START_RED_DOWN(new Pose2d(-36, -60, Math.toRadians(90))),
        START_BLUE_UP(new Pose2d(12, 60, Math.toRadians(270))),
        START_BLUE_DOWN(new Pose2d(-36, 60, Math.toRadians(270)));

        private final Pose2d pose;

        Position(Pose2d pose) {
            this.pose = pose;
        }

        public Pose2d getPose() {
            return pose;
        }
    }

    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        AddTrajectorySequenceCallback blueUpFull = (drive) ->
            drive.trajectorySequenceBuilder(Position.START_BLUE_UP.getPose())
                        .splineTo(new Vector2d(0, 43), Math.toRadians(235))
                        .waitSeconds(2)
                        .turn(Math.toRadians(-75))
                        .splineTo(new Vector2d(-60, 60), Math.toRadians(180))
                        .waitSeconds(2)
                        .back(120)
                        .build();
        AddTrajectorySequenceCallback redUpFull = (drive) ->
                drive.trajectorySequenceBuilder(Position.START_RED_UP.getPose())
                        .splineTo(new Vector2d(0, -43), Math.toRadians(120))
                        .waitSeconds(2)
                        .turn(Math.toRadians(75))
                        .splineTo(new Vector2d(-60, -60), Math.toRadians(180))
                        .waitSeconds(2)
                        .back(120)
                        .build();

        AddTrajectorySequenceCallback blueDownFull = (drive) ->
                drive.trajectorySequenceBuilder(Position.START_BLUE_DOWN.getPose())
                        .splineTo(new Vector2d(-24, 40), Math.toRadians(315))
                        .waitSeconds(2)
                        .turn(Math.toRadians(-135))
                        .splineTo(new Vector2d(-60, 60), Math.toRadians(180))
                        .waitSeconds(2)
                        .back(120)
                        .build();
        AddTrajectorySequenceCallback redDownFull = (drive) ->
                drive.trajectorySequenceBuilder(Position.START_RED_DOWN.getPose())
                        .splineTo(new Vector2d(-24, -40), Math.toRadians(45))
                        .waitSeconds(2)
                        .turn(Math.toRadians(135))
                        .splineTo(new Vector2d(-60, -60), Math.toRadians(180))
                        .waitSeconds(2)
                        .back(120)
                        .build();

        AddTrajectorySequenceCallback blueUpFullLinear = (drive) ->
                drive.trajectorySequenceBuilder(Position.START_BLUE_UP.getPose())
                        .turn(Math.toRadians(-35))
                        .forward(20)
                        .waitSeconds(2)
                        .turn(Math.toRadians(-70))
                        .forward(58)
                        .turn(Math.toRadians(15))
                        .forward(5)
                        .waitSeconds(2)
                        .back(120)
                        .build();
        AddTrajectorySequenceCallback redUpFullLinear = (drive) ->
                drive.trajectorySequenceBuilder(Position.START_RED_UP.getPose())
                        .turn(Math.toRadians(35))
                        .forward(20)
                        .waitSeconds(2)
                        .turn(Math.toRadians(70))
                        .forward(58)
                        .turn(Math.toRadians(-15))
                        .forward(5)
                        .waitSeconds(2)
                        .back(120)
                        .build();

        AddTrajectorySequenceCallback blueDownFullLinear = (drive) ->
                drive.trajectorySequenceBuilder(Position.START_BLUE_DOWN.getPose())
                        .turn(Math.toRadians(35))
                        .forward(23)
                        .waitSeconds(2)
                        .turn(Math.toRadians(-150))
                        .forward(40)
                        .turn(Math.toRadians(25))
                        .forward(2)
                        .waitSeconds(2)
                        .back(120)
                        .build();
        AddTrajectorySequenceCallback redDownFullLinear = (drive) ->
                drive.trajectorySequenceBuilder(Position.START_RED_DOWN.getPose())
                        .turn(Math.toRadians(-35))
                        .forward(23)
                        .waitSeconds(2)
                        .turn(Math.toRadians(150))
                        .forward(40)
                        .turn(Math.toRadians(-25))
                        .forward(2)
                        .waitSeconds(2)
                        .back(120)
                        .build();

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(360), Math.toRadians(360), 12.132362205)
                .followTrajectorySequence(blueUpFull);

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}