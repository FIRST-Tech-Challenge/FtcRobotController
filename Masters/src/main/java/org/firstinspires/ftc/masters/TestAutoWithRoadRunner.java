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
    RobotClass robot;

    @Override
    public void runOpMode() throws InterruptedException {
      //  robot = new RobotClass(hardwareMap,telemetry,this);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(new Vector2d(-36, -63),Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;


        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
             .lineToLinearHeading(new Pose2d(new Vector2d(-34, -39), Math.toRadians(45)))
                .build();
        drive.followTrajectorySequence(trajSeq);

        drive.distanceSensorStuff();
       // robot.distanceSensorStuff();

             Pose2d position= drive.getLocalizer().getPoseEstimate();
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(position)
                .lineToLinearHeading(new Pose2d(new Vector2d(-58.25, -58.25), Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(trajSeq2);

        drive.jevilTurnCarousel(.5,5);
       // robot.jevilTurnCarousel(.5,5);

        Pose2d position2= drive.getLocalizer().getPoseEstimate();
        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(position2)
                .lineToLinearHeading(new Pose2d(new Vector2d(-52, -62), Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(trajSeq3);

        Pose2d position3= drive.getLocalizer().getPoseEstimate();
        TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(position3)
                .lineToLinearHeading(new Pose2d(new Vector2d(-34, -39), Math.toRadians(45)))
                .build();
        drive.followTrajectorySequence(trajSeq4);

        drive.distanceSensorStuff(); 
      //  robot.distanceSensorStuff();

        Pose2d position4= drive.getLocalizer().getPoseEstimate();
        TrajectorySequence trajSeq5 = drive.trajectorySequenceBuilder(position4)
                .lineToLinearHeading(new Pose2d(new Vector2d(-61, -34), Math.toRadians(0)))
                .build();
        drive.followTrajectorySequence(trajSeq5);
    }



}
