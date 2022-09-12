package com.example.meepmeeppathvisualizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import org.jetbrains.annotations.NotNull;

public class MeepMeepTest {
    public static void main(String[] args){

        MeepMeep mm = new MeepMeep(800,90);
        new MeepMeepPersistence(mm);


        MeepMeepPersistence persist = new MeepMeepPersistence(mm);
        persist.restore();

        Pose2d startPose = new Pose2d(-12, -59, Math.toRadians(90));

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

                .setColorScheme(new ColorSchemeRedDark())


                .setDimensions(13, 16)


                .followTrajectorySequence(drive -> {
                    TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);

                    TrajectoryVelocityConstraint STVC = new TrajectoryVelocityConstraint() {
                        @Override
                        public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
                            return 5;
                        }
                    };

                    TrajectoryVelocityConstraint STVE = (a, b, c, d) -> 5;
                    TrajectoryVelocityConstraint SpTVC = (a, b, c, d) -> 1000;
                    TrajectoryAccelerationConstraint SpTAC = (a, b, c, d) -> 1000;

                    builder.forward(15);

                    builder.waitSeconds(1.5);
                    builder.lineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(40)));
                    builder.waitSeconds(3);
                    builder.lineToLinearHeading(new Pose2d(-57, -57, Math.toRadians(90)));
                    builder.strafeRight(15, STVE);
                    builder.lineToLinearHeading(new Pose2d(-12.3, -44, Math.toRadians(90)));
                    builder.waitSeconds(2);
                    builder.lineToLinearHeading(new Pose2d(-60, -34, Math.toRadians(180)));
                    builder.splineTo(new Vector2d(5, 10), 2, SpTVC, SpTAC);





                    builder.setReversed(true);


                    return builder.build();
                });


        mm.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(bot)
                .start();
    }
}
