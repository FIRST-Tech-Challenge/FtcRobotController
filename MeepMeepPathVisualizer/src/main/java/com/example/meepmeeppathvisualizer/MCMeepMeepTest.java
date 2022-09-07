package com.example.meepmeeppathvisualizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class MCMeepMeepTest {

    MeepMeep testM = new MeepMeep(800);
    MeepMeepPersistence persist = new MeepMeepPersistence(testM);


    Pose2d startPose = new Pose2d(0,0,0);

    RoadRunnerBotEntity testBot = new DefaultBotBuilder(testM)
            .setStartPose(startPose)
            .setConstraints(
                    54, // Retrieves constants from the 'Drive Constants' file
                    54,
                    Math.toRadians(63.0254), //MAX_ANG_VEL
                    Math.toRadians(63.0254), //MAX_ANG_ACCEL
                    14.5
            )
            .setColorScheme(new ColorSchemeBlueDark())

            .setDimensions(12,16)



}
