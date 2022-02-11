package org.firstinspires.ftc.Team19567.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.Team19567.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.Team19567.trajectorysequence.TrajectorySequence;

@Config
public class RoadrunnerSequences {
    public TrajectorySequence desiredSequence;

    public TrajectorySequence GetSequence(ROADRUNNER_SEQUENCE roadrunnerSequence,
                                                 SampleMecanumDriveCancelable chassis, Mechanisms mechanisms) {
        switch(roadrunnerSequence) {
            case LINE_TO_SPLINE_WAREHOUSE: {
                desiredSequence = chassis.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .addSpatialMarker(new Vector2d(6, 1), () -> {
                            mechanisms.rotateArm(Utility_Constants.THIRD_LEVEL_POS, 0.7);
                        })
                        .lineToSplineHeading(new Pose2d(27, 6, Math.toRadians(225))).build();
                break;
            }
            case LINE_TO_SPLINE_CAROUSEL: {
                //TODO: WRITE THIS STUFF
            }
            case SPLINE_TO_WAREHOUSE: {
                desiredSequence = chassis.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .addSpatialMarker(new Vector2d(6, 1), () -> {
                            mechanisms.rotateArm(Utility_Constants.THIRD_LEVEL_POS, 0.7);
                        })
                        .lineToSplineHeading(new Pose2d(27, 6, Math.toRadians(225))).build();
                break;
            }
            case BACK_AND_FORTH: {
                break;
            }
        }
        return desiredSequence;

    }
}
