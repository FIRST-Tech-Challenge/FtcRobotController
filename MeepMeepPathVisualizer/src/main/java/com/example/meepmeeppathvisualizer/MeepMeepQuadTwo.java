package com.example.meepmeeppathvisualizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class MeepMeepQuadTwo {
    public static void main(String[] args){
        int aprilTagsId = 3;

        MeepMeep mm = new MeepMeep(800,120);
        new MeepMeepPersistence(mm);
        MeepMeepPersistence persist = new MeepMeepPersistence(mm);
        persist.restore();
        Pose2d startPose = new Pose2d(35, 61.8, Math.toRadians(270));

        // Creating bot
        RoadRunnerBotEntity bot = new DefaultBotBuilder(mm)
                .setStartPose(startPose)
                .setConstraints(
                        54,
                        54,
                        Math.toRadians(63.0254), //MAX_ANG_VEL
                        Math.toRadians(63.0254), //MAX_ANG_ACCEL
                        14.5
                )
                .setColorScheme(new ColorSchemeBlueDark())
                .setDimensions(13, 16)

                .followTrajectorySequence(drive -> {
                    TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);

                    builder.lineToLinearHeading(new Pose2d(32,8,Math.toRadians(225)));
                    builder.waitSeconds(1);
                    loop(builder, 4);

                    if(aprilTagsId == 1){
                        builder.lineTo(new Vector2d(59, 35));
                        builder.waitSeconds(1);
                    }
                    else if(aprilTagsId == 2){
                        builder.lineTo(new Vector2d(35, 8));
                        builder.waitSeconds(1);
                    }
                    else{
                        builder.lineTo(new Vector2d(11.8, 8));
                        builder.waitSeconds(1);
                    }
                    return builder.build();
                });


        mm.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(bot)
                .start();
    }

    public static void loop(TrajectorySequenceBuilder builder, int LoopAmount){
        for(int i = 0; i <= LoopAmount; i++)
        {
            builder.lineToLinearHeading(new Pose2d(57, 12.3, Math.toRadians(0)));
            builder.waitSeconds(2.5); //This would be replaced with an actual intake function
            builder.lineToLinearHeading(new Pose2d(32, 8, Math.toRadians(225)));
            builder.waitSeconds(1);//Under the impression that using the async PID, the slides will be already be moved up
        }
    }
}