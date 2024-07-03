package com.example.meepmeeptesting;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args){
        MeepMeep meepMeep = new MeepMeep(800);


                    Pose2d pose = new Pose2d(12,60,Math.toRadians(270));
                    RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                            // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                            .setConstraints(47.25, 47.25, Math.toRadians(22.36), Math.toRadians(22.36), 13.9)
                            // Option: Set theme. Default = ColorSchemeRedDark()
                            .setColorScheme(new ColorSchemeRedDark())
                            .followTrajectorySequence(drive ->
                                    drive.trajectorySequenceBuilder(new Pose2d(-32, 62, Math.toRadians(270)))
                                            .lineTo(new Vector2d(-36, 62))
                                            .lineToLinearHeading(new Pose2d(-37,33))
                                            .forward(13)
                                            .back(15)
                                            .splineToLinearHeading(new Pose2d(38,36,Math.toRadians(180)),Math.toRadians(50))
                                            .back(10)
                                            .build()

                            );
                    meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                            .setDarkMode(true)
                            // Background opacity from 0-1
                            .setBackgroundAlpha(0.95f)
                            .addEntity(myBot)
                            .start();
                }
        // Set field image

    }
