package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
// import the poses
import static com.example.meepmeeptesting.MeepMeepTesting.*;

public class Bots {


    public static RoadRunnerBotEntity coloredStraysCycleBot() {
        return new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(coloredSampleStartPose)
                        // LOOP 1
                        // go to pickup right
                        .lineToSplineHeading(new Pose2d(
                                -48 * redPoseAdjustment,
                                42 * redPoseAdjustment,
                                Math.toRadians(-90 + redAngleAdjustment)
                        ))
                        .addDisplacementMarker(() -> {
                            // pick up sample
                        })

                        // go to dropoff
                        .setTangent(Math.toRadians(0 + redAngleAdjustment))
                        .splineToSplineHeading(
                                dropSamplePose,
                                Math.toRadians(10 + redAngleAdjustment)
                        )
                        .addDisplacementMarker(() -> {
                            // drop off sample
                        })

                        // go to pickup middle
                        .setTangent(Math.toRadians(180 + redAngleAdjustment))
                        .splineToLinearHeading(new Pose2d(
                                        -40 * redPoseAdjustment,
                                        27 * redPoseAdjustment,
                                        Math.toRadians(180 + redAngleAdjustment)
                                ),
                                Math.toRadians(235 + redAngleAdjustment)
                        )
                        .addDisplacementMarker(() -> {
                            // pick up sample
                        })
                        // go to dropoff
                        .setTangent(Math.toRadians(45 + redAngleAdjustment))
                        .splineToSplineHeading(
                                dropSamplePose,
                                Math.toRadians(0 + redAngleAdjustment)
                        )
                        .addDisplacementMarker(() -> {
                            // drop off sample
                        })

                        // go to pickup left
                        .setTangent(Math.toRadians(165 + redAngleAdjustment))
                        .splineToSplineHeading(new Pose2d(
                                        -52 * redPoseAdjustment,
                                        27 * redPoseAdjustment,
                                        Math.toRadians(180 + redAngleAdjustment)
                                ),
                                Math.toRadians(210 + redAngleAdjustment)
                        )
                        .addDisplacementMarker(() -> {
                            // pick up sample
                        })
                        // go to dropoff
                        .setTangent(Math.toRadians(45 + redAngleAdjustment))
                        .splineToSplineHeading(
                                dropSamplePose,
                                Math.toRadians(0 + redAngleAdjustment)
                        )
                        .addDisplacementMarker(() -> {
                            // drop off sample
                        })
                        .build());
    }

    public static RoadRunnerBotEntity neutralStraysCycleBot() {
        return new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(bucketStartPose)
                        // LOOP 1
                        // go to pickup
                        .lineToSplineHeading(new Pose2d(
                                48 * redPoseAdjustment,
                                39 * redPoseAdjustment,
                                Math.toRadians(-90 + redAngleAdjustment)
                        ))
                        .addDisplacementMarker(() -> {
                            // pick up sample
                        })

                        // go to dropoff
                        .lineToSplineHeading(dropSamplePose)
                        .addDisplacementMarker(() -> {
                            // drop off sample
                        })

                        // LOOP 2
                        .lineToLinearHeading(new Pose2d(
                                58 * redPoseAdjustment,
                                43 * redPoseAdjustment,
                                Math.toRadians(-90 + redAngleAdjustment)
                        ))
                        .addDisplacementMarker(() -> {
                            // pick up sample
                        })
                        .lineToSplineHeading(dropSamplePose)
                        .addDisplacementMarker(() -> {
                            // drop off sample
                        })

                        // LOOP 3
                        .lineToLinearHeading(new Pose2d(
                                52 * redPoseAdjustment,
                                27 * redPoseAdjustment,
                                Math.toRadians(0 + redAngleAdjustment)
                        ))
                        .addDisplacementMarker(() -> {
                            // pick up sample
                        })
                        .lineToSplineHeading(dropSamplePose)
                        .addDisplacementMarker(() -> {
                            // drop off sample
                        })
                        .build());
    }

    public static RoadRunnerBotEntity submersibleCycleBot() {
        return new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(bucketStartPose)
                        // go to pickup
                        .splineToLinearHeading(
                                submersiblePickUpPose,
                                Math.toRadians(-90 + redAngleAdjustment)
                        )
                        .addDisplacementMarker(() -> {
                            // pick up sample
                        })
                        // go to dropoff
//                        .turn(Math.toRadians(-90))
                        .setTangent(Math.toRadians(90 + redAngleAdjustment))
                        .splineToLinearHeading(
                                dropSamplePose,
                                Math.toRadians(30 + redAngleAdjustment)
                        )
                        .addDisplacementMarker(() -> {
                            // drop off sample
                        })

                        // LOOP 1
                        // go back to pickup
                        .setTangent(Math.toRadians(180 + 20 + redAngleAdjustment))
                        .splineToLinearHeading(
                                submersiblePickUpPose,
                                Math.toRadians(-110 + redAngleAdjustment)
                        )
                        .addDisplacementMarker(() -> {
                            // pick up sample
                        })
                        // go back dropoff
                        .setTangent(Math.toRadians(90 + redAngleAdjustment))
                        .splineToLinearHeading(
                                dropSamplePose,
                                Math.toRadians(30 + redAngleAdjustment)
                        )
                        .addDisplacementMarker(() -> {
                            // drop off sample
                        })

                        // LOOP 2
                        // go back to pickup
                        .setTangent(Math.toRadians(180 + 20 + redAngleAdjustment))
                        .splineToLinearHeading(submersiblePickUpPose,
                                Math.toRadians(-110 + redAngleAdjustment)
                        )
                        .addDisplacementMarker(() -> {
                            // pick up sample
                        })
                        // go back dropoff
                        .setTangent(Math.toRadians(90 + redAngleAdjustment))
                        .splineToLinearHeading(
                                dropSamplePose,
                                Math.toRadians(30 + redAngleAdjustment)
                        )
                        .addDisplacementMarker(() -> {
                            // drop off sample
                        })
                        .build());
    }
}
