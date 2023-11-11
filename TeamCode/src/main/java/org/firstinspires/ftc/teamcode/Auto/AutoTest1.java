package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/* velocity + acceleration limit command in trajectory movements:
SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
 */

@Autonomous(name = "Novak Djokovic will reach a record extending 400 weeks at #1")
public class AutoTest1 extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // start pose with random estimated values; will change later
        // we can also make 4 different pose2d values for each of the start positions so changing at start will be easier
        Pose2d startPose = new Pose2d(36, 60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);


        // move robot to pixel stash (will change values later)
        Trajectory moveToStash = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(60, 0), Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        // move robot to the pixel board
        Trajectory moveToBoard = drive.trajectoryBuilder(moveToStash.end())
                .splineTo(new Vector2d(-60, 0), Math.toRadians(-180), SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        // test: move robot backwards back to the stash
        Trajectory moveBack = drive.trajectoryBuilder(moveToBoard.end(), true)
                .splineTo(new Vector2d(36, 0), 0, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .strafeLeft(60)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(moveToStash);
        drive.followTrajectory(moveToBoard);
        drive.followTrajectory(moveBack);
    }
}
