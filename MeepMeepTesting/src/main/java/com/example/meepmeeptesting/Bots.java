package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Bots {
    private static final Pose2d bucketStartPose = new Pose2d(35, 62, Math.toRadians(-90));
    private static final Pose2d coloredSampleStartPose = new Pose2d(-35, 62, Math.toRadians(-90));
    private static final Pose2d submersiblePickUpPose = new Pose2d(27, 0, Math.toRadians(180));
    private static final Pose2d dropSamplePose = new Pose2d(50, 50, Math.toRadians(45));

    public static RoadRunnerBotEntity coloredStraysCycleBot(MeepMeep meepMeep, int endHeading) {
        return new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(bucketStartPose)
                        .waitSeconds(1)
                        // LOOP 1
                        // go to pickup
                        .lineToConstantHeading(new Vector2d(48, 43))
                        .addDisplacementMarker(() -> {
                            // pick up sample
                        })

                        // go to dropoff
                        .lineToSplineHeading(dropSamplePose)
                        .addDisplacementMarker(() -> {
                            // drop off sample
                        })

                        // LOOP 2
                        .lineToLinearHeading(new Pose2d(58, 43, Math.toRadians(-90)))
                        .addDisplacementMarker(() -> {
                            // pick up sample
                        })
                        .lineToSplineHeading(dropSamplePose)
                        .addDisplacementMarker(() -> {
                            // drop off sample
                        })

                        // LOOP 3
                        .lineToLinearHeading(new Pose2d(52, 27, Math.toRadians(0)))
                        .addDisplacementMarker(() -> {
                            // pick up sample
                        })
                        .lineToSplineHeading(dropSamplePose)
                        .addDisplacementMarker(() -> {
                            // drop off sample
                        })
                        .build());
    }

    public static RoadRunnerBotEntity neutralStraysCycleBot(MeepMeep meepMeep, int endHeading) {
        return new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(bucketStartPose)
                        .waitSeconds(1)
                        // LOOP 1
                        // go to pickup
                        .lineToConstantHeading(new Vector2d(48, 43))
                        .addDisplacementMarker(() -> {
                            // pick up sample
                        })

                        // go to dropoff
                        .lineToSplineHeading(dropSamplePose)
                        .addDisplacementMarker(() -> {
                            // drop off sample
                        })

                        // LOOP 2
                        .lineToLinearHeading(new Pose2d(58, 43, Math.toRadians(-90)))
                        .addDisplacementMarker(() -> {
                            // pick up sample
                        })
                        .lineToSplineHeading(dropSamplePose)
                        .addDisplacementMarker(() -> {
                            // drop off sample
                        })

                        // LOOP 3
                        .lineToLinearHeading(new Pose2d(52, 27, Math.toRadians(0)))
                        .addDisplacementMarker(() -> {
                            // pick up sample
                        })
                        .lineToSplineHeading(dropSamplePose)
                        .addDisplacementMarker(() -> {
                            // drop off sample
                        })
                        .build());
    }

    public static RoadRunnerBotEntity submersibleCycleBot(MeepMeep meepMeep, int endHeading) {
        return new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(bucketStartPose)
                        // go to pickup
                        .splineToLinearHeading(submersiblePickUpPose, Math.toRadians(270))
                        .addDisplacementMarker(() -> {
                            // pick up sample
                        })
                        // go to dropoff
                        .turn(Math.toRadians(-90))
                        .splineToLinearHeading(dropSamplePose, Math.toRadians(50))
                        .addDisplacementMarker(() -> {
                            // drop off sample
                        })

                        // LOOP 1
                        // go back to pickup
                        .turn(Math.toRadians(150))
                        .splineToLinearHeading(submersiblePickUpPose, Math.toRadians(270))
                        .addDisplacementMarker(() -> {
                            // pick up sample
                        })
                        // go back to dropoff
                        .turn(Math.toRadians(-90))
                        .splineToLinearHeading(dropSamplePose, Math.toRadians(50))
                        .addDisplacementMarker(() -> {
                            // drop off sample
                        })

                        // LOOP 2
                        // go back to pickup
                        .turn(Math.toRadians(150))
                        .splineToLinearHeading(submersiblePickUpPose, Math.toRadians(270))
                        .addDisplacementMarker(() -> {
                            // pick up sample
                        })
                        // go back to dropoff
                        .turn(Math.toRadians(-90))
                        .splineToLinearHeading(dropSamplePose, Math.toRadians(50))
                        .addDisplacementMarker(() -> {
                            // drop off sample
                        })
                        .build());
    }
}
