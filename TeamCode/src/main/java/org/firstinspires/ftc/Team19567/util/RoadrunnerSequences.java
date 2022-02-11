package org.firstinspires.ftc.Team19567.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.Team19567.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.Team19567.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RoadrunnerSequences {
    public static TrajectorySequence GetSequence(ROADRUNNER_SEQUENCE roadrunnerSequence,
                                                 SampleMecanumDriveCancelable chassis, Mechanisms mechanisms) {
        return chassis.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .addSpatialMarker(new Vector2d(6, 1), () -> {
                    mechanisms.rotateArm(Utility_Constants.THIRD_LEVEL_POS, 0.7);
                })
                .lineToSplineHeading(new Pose2d(27, 6, Math.toRadians(225))).build();

    }
}
