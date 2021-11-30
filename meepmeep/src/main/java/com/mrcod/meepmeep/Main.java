package com.mrcod.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.SequenceSegment;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.LinkedList;
import java.util.List;

public class Main {
    public static void main(String[] args) {
        final Pose2d startPose =  new Pose2d(-40.00,65.00, 0);

        System.setProperty("sun.java2d.opengl", "true");
        System.out.println(System.getProperty("sun.java2d.opengl"));
        MeepMeep meep = new MeepMeep(810);
        meep.setAxesInterval(10);
        meep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK);
        ColorSchemeRedDark scheme = new ColorSchemeRedDark();
        meep.setTheme(scheme);
        meep.setBackgroundAlpha(1);
        RoadRunnerBotEntity bot = new RoadRunnerBotEntity(meep,
                new Constraints(DriveConstants.MAX_VEL, DriveConstants.MAX_ACCEL,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL,
                        DriveConstants.TRACK_WIDTH),
                10D, 14.1D, startPose, scheme, 1D,
                DriveTrainType.MECANUM, true
        );
        meep.addEntity(bot);

        TrajectorySequenceBuilder builder = new TrajectorySequenceBuilder(startPose,
                new MecanumVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL),
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL);

        builder.lineToSplineHeading(new Pose2d(-60, 60));

        bot.followTrajectorySequence(builder.build());

        meep.start();
    }
}
