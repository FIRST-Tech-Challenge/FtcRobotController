package org.firstinspires.ftc.masters.world.paths;

import static org.firstinspires.ftc.masters.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.masters.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.masters.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;

public class BlueFarSidePath {

    public static TrajectorySequence getRightPurple(SampleMecanumDrive drive, Pose2d startPose) {

            return drive.trajectorySequenceBuilder(startPose)
                    .setTangent(Math.toRadians(-50))
                    .splineToLinearHeading(new Pose2d(-47, 15,Math.toRadians(-90)),Math.toRadians(-110))
                    .build();
    }
    public static TrajectorySequence getLeftPurple (SampleMecanumDrive drive,  Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-110))
                .splineToLinearHeading(new Pose2d(-38, 35, Math.toRadians(-180)), Math.toRadians(-30))

                .build();
    }

    public static TrajectorySequence getMidPurple(SampleMecanumDrive drive,  Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-50))
                .splineToLinearHeading(new Pose2d(-45, 14, Math.toRadians(-120)), Math.toRadians(-110))

                .build();
    }

    public static TrajectorySequence getRightPurpleToStack(SampleMecanumDrive drive, Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-57.5, 11.5, Math.toRadians(180)), Math.toRadians(180))
                .build();
    }

    public static TrajectorySequence getLeftPurpleToStack(SampleMecanumDrive drive, Pose2d startPose) {

        return  drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-57.5, 11.5, Math.toRadians(180)), Math.toRadians(180))
                .build();
    }

    public static TrajectorySequence getMidPurpleToStack(SampleMecanumDrive drive, Pose2d startPose) {

        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-140))
                .splineToLinearHeading(new Pose2d(-57.5, 11.5, Math.toRadians(180)), Math.toRadians(180))
                .build();
    }

    public static TrajectorySequence getStackToRightYellow(SampleMecanumDrive drive, Pose2d startPose) {

        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50, 30, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    public static TrajectorySequence getStackToMidYellow(SampleMecanumDrive drive, Pose2d startPose) {

        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(20, 11.5, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50, 36, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    public static TrajectorySequence getStackToLeftYellow(SampleMecanumDrive drive, Pose2d startPose) {

        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(30, 11.5, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50, 38, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    public static TrajectorySequence toStackFromBackboardGate (SampleMecanumDrive drive, Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-57, 11.5, Math.toRadians(180)), Math.toRadians(180))
                .build();
    }

    public static TrajectorySequence toBackboardGate(SampleMecanumDrive drive, Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(47, 28, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    public static TrajectorySequence toStackGate(SampleMecanumDrive drive, Pose2d startPose) {

        return drive.trajectorySequenceBuilder(startPose)

                .lineToLinearHeading(new Pose2d(-61, -10, Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(30, MAX_ANG_VEL, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .build();
    }

    public static TrajectorySequence toStackWing(SampleMecanumDrive drive, Pose2d startPose) {

        return drive.trajectorySequenceBuilder(startPose)

                .lineToLinearHeading(new Pose2d(-61, -34, Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(30, MAX_ANG_VEL, TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .build();
    }

    public static TrajectorySequence park(SampleMecanumDrive drive, Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(44, 15, Math.toRadians(180)), Math.toRadians(0))
                .build();



    }

}
