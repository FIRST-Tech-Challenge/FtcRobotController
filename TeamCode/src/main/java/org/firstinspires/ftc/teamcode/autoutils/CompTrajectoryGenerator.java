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
    /*
    public final TrajectorySequence BLUE_BOTTOM;
    public final TrajectorySequence BLUE_TOP;
    public final TrajectorySequence RED_BOTTOM;
    public final TrajectorySequence RED_TOP;

     */

    private enum trajectories {
        BLUE_BOTTOM,
        BLUE_TOP,
        RED_BOTTOM,
        RED_TOP
    }

    public CompTrajectoryGenerator(SampleMecanumDrive drive, HashMap<Other, DynamicTypeValue> other) {
        DRIVE = drive;
    }

    public static TrajectorySequence generateFieldTrajectory(final SampleMecanumDrive drive, final trajectories trajectory) {
        switch (trajectory) {
            case BLUE_BOTTOM:
                return drive.trajectorySequenceBuilder(new Pose2d(-36.00, 70.00, Math.toRadians(270.00)))
                    .lineTo(new Vector2d(-32.64, 25.64))
                    .lineTo(new Vector2d(-36.00, 70.00))
                    .build();

            case BLUE_TOP:
                break;
            case RED_BOTTOM:
                break;
            case RED_TOP:
                break;
            default:
                break;
        }
    }
}
