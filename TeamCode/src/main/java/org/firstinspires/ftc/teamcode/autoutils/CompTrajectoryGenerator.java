package org.firstinspires.ftc.teamcode.autoutils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

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

        this.BLUE_BOTTOM = generateFieldTrajectory(trajectories.BLUE_BOTTOM, other);
        this.BLUE_TOP = generateFieldTrajectory(trajectories.BLUE_TOP, other);
        this.RED_BOTTOM = generateFieldTrajectory(trajectories.RED_BOTTOM, other);
        this.RED_TOP = generateFieldTrajectory(trajectories.RED_TOP, other);
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
                                                             final HashMap<Other, DynamicTypeValue> other) {
        switch (trajectory) {
            case BLUE_BOTTOM:
                return DRIVE.trajectorySequenceBuilder(new Pose2d(-36.00, 70.00, Math.toRadians(270.00)))
                    .lineTo(new Vector2d(-32.64, 25.64))
                    .lineTo(new Vector2d(-36.00, 70.00))
                    .build();

            case BLUE_TOP:
                return DRIVE.trajectorySequenceBuilder(new Pose2d(12.00, 70.00, Math.toRadians(266.91)))
                    .lineTo(new Vector2d(12.00, 25.00))
                    .lineTo(new Vector2d(12.00, 70.00))
                    .build();

            case RED_BOTTOM:
                return DRIVE.trajectorySequenceBuilder(new Pose2d(12.00, -70.00, Math.toRadians(93.09)))
                    .lineTo(new Vector2d(12.00, -25.00))
                    .lineTo(new Vector2d(12.00, -70.00))
                    .build();

            case RED_TOP:
                return DRIVE.trajectorySequenceBuilder(new Pose2d(-36.00, -70.00, Math.toRadians(90.00)))
                        .lineTo(new Vector2d(-36.00, -25.00))
                        .lineTo(new Vector2d(-36.00, -70.00))
                        .build();

            default:
                return null;
        }
    }
}
