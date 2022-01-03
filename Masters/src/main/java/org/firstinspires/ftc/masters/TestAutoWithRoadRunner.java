package org.firstinspires.ftc.masters;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.drive.DriveConstants;
import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;

@Autonomous(name="test auto with roadRunner")
public class TestAutoWithRoadRunner extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d();

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;


            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                   // .lineToConstantHeading(new Vector2d(24, 12), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
.lineToLinearHeading(new Pose2d(new Vector2d(24, 12), Math.toRadians(45)))
                    //.splineTo(new Vector2d(12, 12),0)
                    //.splineTo(new Vector2d(12, 0), 0)
                    .build();
            drive.followTrajectorySequence(trajSeq);

            drive.getLocalizer().getPoseEstimate();

    }



}
