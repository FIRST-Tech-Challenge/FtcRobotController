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
                .splineToLinearHeading(new Pose2d(-38.5, 33, Math.toRadians(-180)), Math.toRadians(-30))

                .build();
    }

    public static TrajectorySequence getMidPurple(SampleMecanumDrive drive,  Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-50))
                .splineToLinearHeading(new Pose2d(-47, 12, Math.toRadians(-140)), Math.toRadians(-110))

                .build();
    }

    public static TrajectorySequence getRightPurpleSushi(SampleMecanumDrive drive,Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-47.5, 38, Math.toRadians(-90)), Math.toRadians(-90))
                .build();
    }

    public static TrajectorySequence getLeftPurpleSushi(SampleMecanumDrive drive, Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-30, 36, Math.toRadians(310)), Math.toRadians(-40))
                .build();
    }
    public static TrajectorySequence getMidPurpleSushi(SampleMecanumDrive drive, Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-34, 34, Math.toRadians(-90)), Math.toRadians(-70))
                .build();
    }


    public static TrajectorySequence parkOutside(SampleMecanumDrive drive, Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(44, 58, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }



//    public static TrajectorySequence getLevelPurpleSushi(SampleMecanumDrive drive,Pose2d startPose){
//
//    }
//
//    public static TrajectorySequence getMidPurpleSushi(SampleMecanumDrive drive,Pose2d startPose){
//
//    }

    public static TrajectorySequence getRightPurpleToStack(SampleMecanumDrive drive, Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-58.75, 11.5, Math.toRadians(180)), Math.toRadians(180))
                .build();
    }

    public static TrajectorySequence getLeftPurpleToStack(SampleMecanumDrive drive, Pose2d startPose) {

        return  drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-59.25, 11.75, Math.toRadians(180)), Math.toRadians(180))
                .build();
    }

    public static TrajectorySequence getMidPurpleToStack(SampleMecanumDrive drive, Pose2d startPose) {

        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-140))
                .splineToLinearHeading(new Pose2d(-58.75, 11.5, Math.toRadians(180)), Math.toRadians(180))
                .build();
    }

    public static TrajectorySequence getStackToRightYellow(SampleMecanumDrive drive, Pose2d startPose) {

        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50, 27, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    public static TrajectorySequence getStackToMidYellow(SampleMecanumDrive drive, Pose2d startPose) {

        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(30, 11.5, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50, 29, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    public static TrajectorySequence getStackToLeftYellow(SampleMecanumDrive drive, Pose2d startPose) {

        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(30, 11.5, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50, 34, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    public static TrajectorySequence toGateFromBackdrop(SampleMecanumDrive drive, Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(180))
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(-58, -10.5, Math.toRadians(180)), Math.toRadians(180))
                .build();
    }

    public static TrajectorySequence toStackFromPark(SampleMecanumDrive drive, Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-58, 10.5, Math.toRadians(180)))
                .build();
    }

    public static TrajectorySequence toStackFromGate(SampleMecanumDrive drive, Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                // .setTangent(Math.toRadians(180))
                // .splineToLinearHeading(new Pose2d(10, -11.5, Math.toRadians(180)), Math.toRadians(180))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-58, 10.5, Math.toRadians(180)), Math.toRadians(180))
                .build();
    }


    public static TrajectorySequence toStackFromBackboardGate (SampleMecanumDrive drive, Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(180))
                .setTangent(Math.toRadians(180))
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

//    public static TrajectorySequence park(SampleMecanumDrive drive, Pose2d startPose){
//        return drive.trajectorySequenceBuilder(startPose)
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(44, 12, Math.toRadians(180)), Math.toRadians(0))
//                .build();
//
//    }

    public static TrajectorySequence park(SampleMecanumDrive drive, Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(180))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-57, 11.5, Math.toRadians(180)), Math.toRadians(180))
                .build();
    }

    public static TrajectorySequence rightPurpleToYellow(SampleMecanumDrive drive, Pose2d startPose){
        return  drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(70))
                .splineToLinearHeading(new Pose2d(-30, 57.5, Math.toRadians(180)), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(20, 57.5, Math.toRadians(180)), Math.toRadians(0))

                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48, 22, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    public static TrajectorySequence leftPurpleToYellow(SampleMecanumDrive drive, Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(140))
                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(40))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(20, 58, Math.toRadians(180)), Math.toRadians(0))

                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50, 34.5, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

    public static TrajectorySequence midPurpleToYellow(SampleMecanumDrive drive, Pose2d startPose){
        return drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(150))
                .splineToLinearHeading(new Pose2d(-30, 58, Math.toRadians(180)), Math.toRadians(20))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(20, 58, Math.toRadians(180)), Math.toRadians(0))

                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50, 28, Math.toRadians(180)), Math.toRadians(0))
                .build();
    }

}
