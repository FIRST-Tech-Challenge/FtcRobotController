package com.example.meepmeeppathvisualizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import org.jetbrains.annotations.NotNull;

public class MeepMeepQuadThree {
    public static void main(String[] args){
        int aprilTagsId = 3;
        MeepMeep mm = new MeepMeep(400,90);
        new MeepMeepPersistence(mm);


        MeepMeepPersistence persist = new MeepMeepPersistence(mm);
        persist.restore();

        Pose2d startPose = new Pose2d(-35, 61.8, Math.toRadians(270));

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

                    /*TrajectoryVelocityConstraint STVC = new TrajectoryVelocityConstraint() {
                        @Override
                        public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
                            return 5;
                        }
                    };

                    TrajectoryAccelerationConstraint STAC = new TrajectoryAccelerationConstraint() {
                        @Override
                        public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
                            return 0;
                        }
                    };

                    TrajectoryVelocityConstraint STVE = (a, b, c, d) -> 5;
                    TrajectoryVelocityConstraint SpTVC = (a, b, c, d) -> 1000;
                    TrajectoryAccelerationConstraint SpTAC = (a, b, c, d) -> 1000;
                */
                //Unsureof what the commented code does so I commented it out
                    builder.lineToLinearHeading(new Pose2d(-33,8,Math.toRadians(-39.75)));
                    for(int i=1; i <= 3;i++ )
                        cycle(builder);

                    if(aprilTagsId == 1)
                        builder.lineTo(new Vector2d(-11.6,24.3));
                    else if(aprilTagsId ==2)
                        builder.waitSeconds(2.74);
                    else
                        builder.lineTo(new Vector2d(-58.2,24.6));


                    return builder.build();
                });


        mm.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(bot)
                .start();


    }

    public static void cycle(TrajectorySequenceBuilder builder){
        builder.lineToLinearHeading(new Pose2d(-57,12.3,Math.toRadians(180)));
        builder.waitSeconds(2.5); //This would be replaced with an actual intake function
        builder.lineToLinearHeading(new Pose2d(-33,8,Math.toRadians(-39.75)));
        builder.waitSeconds(1);//Under the impression that using the async PID, the slides will be already be moved up
    }

}

