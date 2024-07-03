package org.firstinspires.ftc.masters.world.paths;

import static org.firstinspires.ftc.masters.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.masters.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.masters.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.masters.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;

public class BlueBackDropPath {

    public static TrajectorySequence getRightPurple(SampleMecanumDrive drive, Pose2d startPose) {

        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-40))
                .splineToLinearHeading(new Pose2d(12, 30, Math.toRadians(180)), Math.toRadians(-70))
                .build();
    }

    public static TrajectorySequence getLeftPurple(SampleMecanumDrive drive, Pose2d startPose) {
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-40))
                .splineToLinearHeading(new Pose2d(33.5, 25, Math.toRadians(180)), Math.toRadians(-70))

                .build();
    }

    public static TrajectorySequence getMidPurple(SampleMecanumDrive drive, Pose2d startPose) {
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(16, 34, Math.toRadians(-90)), Math.toRadians(-90))


                .build();
    }


    public static TrajectorySequence getRightYellow(SampleMecanumDrive drive, Pose2d startPose) {

        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50, 25, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    public static TrajectorySequence getMidYellow(SampleMecanumDrive drive, Pose2d startPose) {

        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(50, 30, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    public static TrajectorySequence getLeftYellow(SampleMecanumDrive drive, Pose2d startPose) {

        return drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50, 41, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    public static TrajectorySequence toStackTruss(SampleMecanumDrive drive, Pose2d startPose) {
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(10, 61, Math.toRadians(181)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-31, 61, Math.toRadians(181)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-44, 33.5, Math.toRadians(180)), Math.toRadians(-140))
                .build();

    }

    public static TrajectorySequence fromStackToBoardTruss(SampleMecanumDrive drive, Pose2d startPose) {
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(40))
                .splineToLinearHeading(new Pose2d(-50, 60.5, Math.toRadians(179)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(25, 60.5, Math.toRadians(179)), Math.toRadians(0))

                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48, 55, Math.toRadians(150)), Math.toRadians(0))
                .build();
    }

    public static TrajectorySequence park(SampleMecanumDrive drive, Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(44, 58, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    public static TrajectorySequence toStackWing(SampleMecanumDrive drive, Pose2d startPose) {

        return drive.trajectorySequenceBuilder(startPose)

                .lineToLinearHeading(new Pose2d(-60.1, 35, Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(30, MAX_ANG_VEL, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .build();
    }

    public static TrajectorySequence toStackGate(SampleMecanumDrive drive, Pose2d startPose) {

        return drive.trajectorySequenceBuilder(startPose)

                .lineToLinearHeading(new Pose2d(-61, 10, Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(30, MAX_ANG_VEL, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .build();
    }

}
