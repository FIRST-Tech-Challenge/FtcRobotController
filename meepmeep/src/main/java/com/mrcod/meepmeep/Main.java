package com.mrcod.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.mrcod.meepmeep.entity.field.CarouselEntity;
import com.mrcod.meepmeep.entity.field.TapeMeasureEntity;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class Main {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep meep = new MeepMeep(810);
        meep.setAxesInterval(10);
        meep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK);
        ColorScheme scheme = new ColorSchemeBlueDark();
        meep.setTheme(scheme);
        meep.setBackgroundAlpha(1);

        RoadRunnerBotEntity storageUnitBot = storageBot(meep);
        RoadRunnerBotEntity warehouseBot = warehouseBot(meep);
        meep.addEntity(storageUnitBot);
        meep.addEntity(new CarouselEntity(meep,
                new Vector2d(-70 + MeepMeepHelper.inchesToCoordinate(2),
                        70 - MeepMeepHelper.inchesToCoordinate(2))));
        meep.addEntity(new CarouselEntity(meep,
                new Vector2d(-70 + MeepMeepHelper.inchesToCoordinate(2),
                        -70 + MeepMeepHelper.inchesToCoordinate(2))));
        //meep.addEntity(warehouseBot);


        storageUnitBot.start();
        warehouseBot.start();

        meep.start();
    }

    public static RoadRunnerBotEntity storageBot(MeepMeep meep) {
        final Pose2d startPose = new Pose2d(-40, 70 - MeepMeepHelper.inchesToCoordinate(9D),
                Math.toRadians(90));

        RoadRunnerBotEntity bot = new RoadRunnerBotEntity(meep,
                new Constraints(DriveConstants.MAX_VEL, DriveConstants.MAX_ACCEL,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL,
                        DriveConstants.TRACK_WIDTH),
                MeepMeepHelper.inchesToCoordinate(12D), MeepMeepHelper.inchesToCoordinate(18D),
                startPose, new ColorSchemeBlueDark(), 1D, DriveTrainType.MECANUM,
                true
        );

        TrajectorySequenceBuilder builder = new TrajectorySequenceBuilder(startPose,
                new MecanumVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL),
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL);

        TapeMeasureEntity tapeMeasure = new TapeMeasureEntity(meep, bot.getPose(), 100, 40);
        tapeMeasure.setZIndex(-1);

        builder.lineTo(new Vector2d(-40, 55));
        builder.splineToLinearHeading(new Pose2d(-19, 40, Math.toRadians(110)), Math.toRadians(-110));
        builder.lineTo(new Vector2d(-19, 45));
        builder.lineToLinearHeading(new Pose2d(-59, 57.5, Math.toRadians(240)));
        builder.lineToLinearHeading(new Pose2d(-60, 35, Math.toRadians(90)));

        builder.addTemporalMarker(() -> {
            Pose2d pose2d = bot.getPose();
            tapeMeasure.setPose(new Pose2d(pose2d.getX(), pose2d.getY(), Math.toRadians(0)));
            tapeMeasure.setExtending(true);
            meep.addEntity(tapeMeasure);
            tapeMeasure.setLength(0);
        });
        builder.waitSeconds(4);
        builder.addTemporalMarker(() -> {
            meep.removeEntity(tapeMeasure);
            tapeMeasure.setExtending(false);
        });
        builder.waitSeconds(0.1);


        bot.followTrajectorySequence(builder.build());

        return bot;
    }


    public static RoadRunnerBotEntity warehouseBot(MeepMeep meep) {
        final Pose2d startPose = new Pose2d(0, 70 - MeepMeepHelper.inchesToCoordinate(9D),
                Math.toRadians(-90));
        RoadRunnerBotEntity bot = new RoadRunnerBotEntity(meep,
                new Constraints(DriveConstants.MAX_VEL, DriveConstants.MAX_ACCEL,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL,
                        DriveConstants.TRACK_WIDTH),
                MeepMeepHelper.inchesToCoordinate(12D), MeepMeepHelper.inchesToCoordinate(18D),
                startPose, new ColorSchemeBlueDark(), 1D, DriveTrainType.MECANUM,
                false
        );

        TrajectorySequenceBuilder builder = new TrajectorySequenceBuilder(startPose,
                new MecanumVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL),
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL);

        return bot;
    }
}
