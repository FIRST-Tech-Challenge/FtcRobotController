package org.firstinspires.ftc.Team19567.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.Team19567.drive.MecanumDriveCancelable;
import org.firstinspires.ftc.Team19567.trajectorysequence.TrajectorySequence;

/**
 * Attempt at creating a file with all of the Roadrunner sequences <br>
 * Note: was never used
 */

@Deprecated
@Config

//TODO: Elaborate on this in the future
public class RoadrunnerSequences {
    public static TrajectorySequence desiredSequence;

    //Function to get all the rr sequences needed
    public static TrajectorySequence GetSequence(ROADRUNNER_SEQUENCE roadrunnerSequence,
                                                 MecanumDriveCancelable chassis, Mechanisms mechanisms) {
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
