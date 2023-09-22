package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import javax.imageio.ImageIO;
import java.io.File;
import java.awt.Image;
import java.io.IOException;

public class MeepMeepTesting {
    enum autos {
        BBHIGH,
        BRHIGH,
        RBHIGH,
        RRHIGH,
        TESTING,
        SMOLTESTING
    };
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(0, 20, 0))
//                                        // 40 by 40
//                                        .strafeRight(18) // 9/10 of 20
//                                        .forward(5)
//                                        .turn(Math.toRadians(-45))
//                                        .forward(5)
//                                        .turn(Math.toRadians(45))
//                                        .forward(10)
//                                        .turn(Math.toRadians(90))
//                                        // Next side
//                                        .forward(10)
//                                        .turn(Math.toRadians(45))
//                                        .forward(5)
//                                        .turn(Math.toRadians(-45))
//                                        .forward(10)
//                                        .turn(Math.toRadians(-45))
//                                        .forward(5)
//                                        .turn(Math.toRadians(45))
//                                        .forward(10)
//                                        .turn(Math.toRadians(90))
//                                        // Next side
//                                        .forward(10)
//                                        .turn(Math.toRadians(45))
//                                        .forward(5)
//                                        .turn(Math.toRadians(-45))
//                                        .forward(10)
//                                        .turn(Math.toRadians(-45))
//                                        .forward(5)
//                                        .turn(Math.toRadians(45))
//                                        .forward(10)
//                                        .turn(Math.toRadians(90))
//                                        // Next side
//                                        .forward(10)
//                                        .turn(Math.toRadians(45))
//                                        .forward(5)
//                                        .turn(Math.toRadians(-45))
//                                        .forward(10)
//                                        .turn(Math.toRadians(-45))
//                                        .forward(5)
//                                        .turn(Math.toRadians(45))
//                                        .forward(10)
//                                        .turn(Math.toRadians(90))
//                                        // Final side
//                                        .forward(10)
//                                        .turn(Math.toRadians(45))
//                                        .forward(5)
//                                        .turn(Math.toRadians(-45))
//                                        .forward(5)
//                                        .strafeLeft(16)
                                        .splineTo(new Vector2d(), 0)
                                        .splineTo(new Vector2d(0,20), 0)
//                                        .strafeRight(10)
//                                        .forward(5)
//                                        .splineTo(new Vector2d(12,20),Math.toRadians(90))
//                                        .forward(5)
//                                        .splineTo(new Vector2d(0,38),Math.toRadians(180))
//                                        .splineTo(new Vector2d(-12,20),Math.toRadians(-90))
//                                        .forward(10)
//                                        .splineTo(new Vector2d(0,-10),Math.toRadians(0))
//                                        .strafeLeft(10)
                                        .build()
                );
        Image img = null;
        try { img = ImageIO.read(new File("images/uqcy8o9sfpob1.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img)

//        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}