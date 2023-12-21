package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class RCInLeft extends MeepMeepBoilerplate {
    @Override
    public TrajectorySequence getTrajectorySequence(Detection detection, DriveShim drive) {
        TrajectorySequence trajectorySequence;
        switch (detection) {
            case LEFT:
                trajectorySequence = drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                        .forward(28.0)
                        .turn(Math.toRadians(90))
                        .forward(8)
//                            .addTemporalMarker(() -> passiveServo.setPosition(0.1))
                        .waitSeconds(.25)
                        .back(32)
                        .waitSeconds(.25)
                        .turn(Math.toRadians(180))
                        .waitSeconds(.25)
                        .strafeLeft(11.5)
                        .waitSeconds(.25)
                        .forward(12.5)
                        .waitSeconds(.25)
//                            .addTemporalMarker(() -> autoServo.setPosition(1))
                        .waitSeconds(1)
//                            .addTemporalMarker(() -> autoServo.setPosition(0.8))
                        .waitSeconds(.5)
                        .back(3)
                        .strafeLeft(14)
                        .forward(10)
                        .waitSeconds(1)
//                            .addTemporalMarker(() -> autoServo.setPosition(0.6))
                        .waitSeconds(1)
                        .build();
                break;
            case CENTER:
                trajectorySequence =
                        drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                                .forward(31.5)
                                .waitSeconds(.25)
//                            .addDisplacementMarker(() -> passiveServo.setPosition(0.1))
                                .waitSeconds(.25)
                                .back(12)
                                .waitSeconds(.25)
                                .turn(Math.toRadians(-90))
                                .waitSeconds(.25)
                                .forward(12)
                                .strafeLeft(11)
                                .forward(24.5)
                                .waitSeconds(.25)
//                            .addTemporalMarker(() -> autoServo.setPosition(1))
                                .waitSeconds(1)
//                            .addTemporalMarker(() -> autoServo.setPosition(0.8))
                                .waitSeconds(.5)
                                .back(3)
                                .strafeLeft(20)
                                .forward(11)
                                .waitSeconds(1)
//                            .addTemporalMarker(() -> autoServo.setPosition(0.6))
                                .waitSeconds(1)
                                .build();
                break;
            case RIGHT:
                trajectorySequence =
                        drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                                .forward(2.0)
                                .waitSeconds(.25)
                                .strafeRight(8)
                                .forward(21)
                                .waitSeconds(.25)
//                            .addTemporalMarker(() -> passiveServo.setPosition(0.1))
                                .waitSeconds(.25)
                                .back(9.5)
                                .waitSeconds(.25)
                                .turn(Math.toRadians(-90))
                                .waitSeconds(.25)
                                .forward(12.5)
                                .waitSeconds(.25)
                                .strafeLeft(13)
                                .waitSeconds(.25)
                                .forward(15.5)
                                .waitSeconds(.5)
//                            .addTemporalMarker(() -> autoServo.setPosition(1))
                                .waitSeconds(1.5)
//                            .addTemporalMarker(() -> autoServo.setPosition(0.9))
                                .waitSeconds(.5)
                                .back(3)
                                .strafeLeft(26)
                                .forward(10)
                                .waitSeconds(1)
//                            .addTemporalMarker(() -> autoServo.setPosition(0.6))
                                .waitSeconds(1)
                                .build();
                break;
            default:
                trajectorySequence = null;
                System.out.println("Nothing detected");
        }

        return trajectorySequence;
    }
}
