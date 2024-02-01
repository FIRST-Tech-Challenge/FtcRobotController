package org.firstinspires.ftc.teamcode.autoutils;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Other.DynamicTypeValue;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer.Other;

import java.util.HashMap;

public class CompTrajectoryGenerator {

    private final SampleMecanumDrive DRIVE;

    /* Pre-made Trajectories */
    public final TrajectorySequence BLUE_BOTTOM;
    public final TrajectorySequence BLUE_TOP;
    public final TrajectorySequence RED_BOTTOM;
    public final TrajectorySequence RED_TOP;

    public enum trajectories {
        BLUE_BOTTOM,
        BLUE_TOP,
        RED_BOTTOM,
        RED_TOP
    }

    public CompTrajectoryGenerator(SampleMecanumDrive drive, final HashMap<Other, DynamicTypeValue> other) {
        DRIVE = drive;

        final boolean PARK_LEFT = false;

        this.BLUE_BOTTOM = generateFieldTrajectory(trajectories.BLUE_BOTTOM, other, PARK_LEFT);
        this.BLUE_TOP = generateFieldTrajectory(trajectories.BLUE_TOP, other, PARK_LEFT);
        this.RED_BOTTOM = generateFieldTrajectory(trajectories.RED_BOTTOM, other, PARK_LEFT);
        this.RED_TOP = generateFieldTrajectory(trajectories.RED_TOP, other, PARK_LEFT);
    }

    public void setStartingPosition(trajectories trajectory) {
        Pose2d pose;
        switch (trajectory) {
            case BLUE_BOTTOM:
                pose = BLUE_BOTTOM.start();
                break;

            case BLUE_TOP:
                pose = BLUE_TOP.start();
                break;

            case RED_BOTTOM:
                pose = RED_BOTTOM.start();
                break;

            case RED_TOP:
                pose = RED_TOP.start();
                break;

            default:
                return;
        }

        DRIVE.setPoseEstimate(pose);
    }

    public TrajectorySequence generateFieldTrajectory(final trajectories trajectory,
                                                             final HashMap<Other, DynamicTypeValue> other,
                                                      final boolean PARK_LEFT) {
        final TrajectoryAccelerationConstraint accelerationConstraint =
                SampleMecanumDrive.getAccelerationConstraint(25);
        final TrajectoryVelocityConstraint velocityConstraint =
                SampleMecanumDrive.getVelocityConstraint(30,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);

        switch (trajectory) {
            case BLUE_BOTTOM:
                if (!PARK_LEFT) {
                    return DRIVE.trajectorySequenceBuilder(new Pose2d(-36.00, 70.00, Math.toRadians(270.00)))
                            .setConstraints(velocityConstraint, accelerationConstraint)
                            .lineTo(new Vector2d(-36.00, 10.00))
                            .lineTo(new Vector2d(58.00, 23.00))
                            .build();
                }
                return DRIVE.trajectorySequenceBuilder(new Pose2d(-36.00, 70.00, Math.toRadians(0)))
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .lineTo(new Vector2d(-36.00, 66.00))
                        .lineTo(new Vector2d(50.00, 66.00))
                        .build();

            case BLUE_TOP:
                if (PARK_LEFT) {
                    return DRIVE.trajectorySequenceBuilder(new Pose2d(12.00, 70.00, Math.toRadians(266.91)))
                            .setConstraints(velocityConstraint, accelerationConstraint)
                            .lineTo(new Vector2d(60, 70))
                            .build();
                }
                return DRIVE.trajectorySequenceBuilder(new Pose2d(12.00, 70.00, Math.toRadians(266.91)))
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .lineTo(new Vector2d(12.00, 20))
                        .lineTo(new Vector2d(60, 20))
                        .build();

            case RED_BOTTOM:
                return DRIVE.trajectorySequenceBuilder(new Pose2d(12.00, -70.00, Math.toRadians(93.09)))
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .lineTo(new Vector2d(12.00, -25.00))
                        .lineTo(new Vector2d(12.00, -70.00))
                        .build();

            case RED_TOP:
                if (PARK_LEFT) {
                    return DRIVE.trajectorySequenceBuilder(new Pose2d(-36.00,  70.00, Math.toRadians(90.00)))
                            .setConstraints(velocityConstraint, accelerationConstraint)
                            .lineTo(new Vector2d(-36.00, -25.00))
                            .lineTo(new Vector2d(-36.00, -70.00))
                            .build();
                }
                return DRIVE.trajectorySequenceBuilder(new Pose2d(-36.00, -70.00, Math.toRadians(90.00)))
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .lineTo(new Vector2d(60, -70.00))
                        .build();

            default:
                return null;
        }
    }
}
