package com.mrcod.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Pose2dKt;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.mrcod.meepmeep.entity.field.CarouselEntity;
import com.mrcod.meepmeep.entity.field.TapeMeasureEntity;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.util.FieldUtil;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import static com.mrcod.meepmeep.MeepMeepHelper.inchesToCoordinate;

public class Main {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep meep = new MeepMeep(720);
        meep.setAxesInterval(10);
        meep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK);
        ColorScheme scheme = new ColorSchemeBlueDark();
        meep.setTheme(scheme);
        meep.setBackgroundAlpha(1);

        RoadRunnerBotEntity warehouseBot = warehouseBot(meep);
        meep.addEntity(warehouseBot);
        RoadRunnerBotEntity storageUnitBot = storageBot(meep, false);
        meep.addEntity(storageUnitBot);
        meep.addEntity(new CarouselEntity(meep,
                new Vector2d(-70 + inchesToCoordinate(2),
                        70 - inchesToCoordinate(2))));
        meep.addEntity(new CarouselEntity(meep,
                new Vector2d(-70 + inchesToCoordinate(2),
                        -70 + inchesToCoordinate(2))));

        meep.start();
    }

    public static RoadRunnerBotEntity storageBot(MeepMeep meep, boolean red) {
        final Pose2d startPose = new Pose2d(-40, (red ? -1 : 1) * 70 - inchesToCoordinate(9D),
                Math.toRadians(red ? 270 : 90));

        RoadRunnerBotEntity bot = new RoadRunnerBotEntity(meep,
                new Constraints(DriveConstants.MAX_VEL, DriveConstants.MAX_ACCEL,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL,
                        DriveConstants.TRACK_WIDTH),
                inchesToCoordinate(12D), inchesToCoordinate(18D),
                startPose, red ? new ColorSchemeRedDark() : new ColorSchemeBlueDark(), 1D,
                DriveTrainType.MECANUM, false
        );
        bot.setZIndex(5);

        TrajectorySequenceBuilder builder = new TrajectorySequenceBuilder(startPose,
                new MecanumVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL),
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL);

        TapeMeasureEntity tapeMeasure = new TapeMeasureEntity(meep, new Pose2d(), 100,
                40);
        tapeMeasure.setZIndex(10);


        builder.waitSeconds(0.1);
        // 9.35 seconds long
        builder.lineTo(new Vector2d(-40, red ? -55 : 55));
        builder.splineToLinearHeading(new Pose2d(-20, red ? -40 : 40, Math.toRadians(-70)),
                Math.toRadians(-110));
        builder.waitSeconds(2);
        builder.lineTo(new Vector2d(-19, red ? -45 : 45));
        builder.lineToLinearHeading(new Pose2d(-59, red ? -57.5 : 57.5, Math.toRadians(red ? 60 : 240)));
        builder.waitSeconds(3);
        builder.lineToLinearHeading(new Pose2d(-60, red ? -35 : 35, Math.toRadians(90)));
        builder.waitSeconds(43.85);
        builder.addDisplacementMarker(() -> {
            Pose2d pose2d = bot.getPose();
            tapeMeasure.setPose(new Pose2d(pose2d.getX() + inchesToCoordinate(6D), pose2d.getY() + 5, Math.toRadians(0)));
            tapeMeasure.setExtending(true);
            meep.addEntity(tapeMeasure);
            tapeMeasure.setZIndex(10);
            tapeMeasure.setLength(0);
        });
        builder.waitSeconds(3.9);
        builder.addDisplacementMarker(() -> {
            tapeMeasure.setLength(0);
            tapeMeasure.setExtending(false);
            meep.removeEntity(tapeMeasure);
        });
        builder.waitSeconds(0.1);

        bot.followTrajectorySequence(builder.build());

        return bot;
    }


    public static RoadRunnerBotEntity warehouseBot(MeepMeep meep) {
        final Pose2d startPose = new Pose2d(0, 70 - inchesToCoordinate(9D),
                Math.toRadians(90));
        RoadRunnerBotEntity bot = new RoadRunnerBotEntity(meep,
                new Constraints(DriveConstants.MAX_VEL, DriveConstants.MAX_ACCEL,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL,
                        DriveConstants.TRACK_WIDTH),
                inchesToCoordinate(12D), inchesToCoordinate(18D),
                startPose, new ColorSchemeBlueDark(), 1D, DriveTrainType.MECANUM,
                false
        );
        bot.setZIndex(5);

        TrajectorySequenceBuilder builder = new TrajectorySequenceBuilder(startPose,
                new MecanumVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL),
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL);

        builder.lineToLinearHeading(new Pose2d(-3, 40, Math.toRadians(70)));
        builder.waitSeconds(2);
        builder.lineToLinearHeading(new Pose2d(-3, 60, Math.toRadians(0)));
        builder.lineTo(new Vector2d(20, 64));
        builder.lineTo(new Vector2d(40, 64));
        for (int i = 0; i < 4; i++) {
            builder.lineTo(new Vector2d(-3,64));
            builder.lineToLinearHeading(new Pose2d(-3, 40, Math.toRadians(70)));
            builder.waitSeconds(2);
            builder.lineToLinearHeading(new Pose2d(-3,64,0));
            builder.lineTo(new Vector2d(40,64));
        }

        bot.followTrajectorySequence(builder.build());

        return bot;
    }
}
