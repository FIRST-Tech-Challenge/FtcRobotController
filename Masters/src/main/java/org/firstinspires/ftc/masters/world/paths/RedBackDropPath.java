package org.firstinspires.ftc.masters.world.paths;

import static org.firstinspires.ftc.masters.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.masters.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.masters.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;

public class RedBackDropPath {

    public static TrajectorySequence getRightPurple(SampleMecanumDrive drive, Pose2d startPose) {

        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-220))
                .splineToLinearHeading(new Pose2d(33, -27, Math.toRadians(180)), Math.toRadians(-250))
                .build();
    }
    public static TrajectorySequence getLeftPurple (SampleMecanumDrive drive,  Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-220))
                .splineToLinearHeading(new Pose2d(10, -30, Math.toRadians(180)), Math.toRadians(-70-180))

                .build();
    }

    public static TrajectorySequence getMidPurple(SampleMecanumDrive drive,  Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians((90)))
                .splineToLinearHeading(new Pose2d(16, -34, Math.toRadians(90)), Math.toRadians(90))

                .build();
    }


    public static TrajectorySequence getRightYellow(SampleMecanumDrive drive, Pose2d startPose) {

        return drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50, -42, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    public static TrajectorySequence getMidYellow(SampleMecanumDrive drive, Pose2d startPose) {

        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(50, -32, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    public static TrajectorySequence getLeftYellow(SampleMecanumDrive drive, Pose2d startPose) {

        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50, -29, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    public static TrajectorySequence toStackTruss(SampleMecanumDrive drive, Pose2d startPose) {
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(10, -62, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-30, -62, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-58, -35.2, Math.toRadians(180)), Math.toRadians(-140-180))
                .build();

    }

    public static TrajectorySequence fromStackToBoardTruss(SampleMecanumDrive drive, Pose2d startPose) {
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(40))
                .splineToLinearHeading(new Pose2d(-45, -62, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(20, -62, Math.toRadians(180)), Math.toRadians(0))

                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48, -38, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    public static TrajectorySequence park(SampleMecanumDrive drive, Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(44, -58, Math.toRadians(180)), Math.toRadians(0))
                .build();

    }

    public static TrajectorySequence toStackWing(SampleMecanumDrive drive, Pose2d startPose) {

        return drive.trajectorySequenceBuilder(startPose)

                .lineToLinearHeading(new Pose2d(-61, -34, Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(30, MAX_ANG_VEL, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .build();
    }


}
