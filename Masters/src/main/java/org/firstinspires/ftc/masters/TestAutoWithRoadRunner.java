package org.firstinspires.ftc.masters;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.drive.DriveConstants;
import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;

import java.util.Date;

@Autonomous(name="test auto with roadRunner")
public class TestAutoWithRoadRunner extends LinearOpMode {
    RobotClass robot;

    @Override
    public void runOpMode() throws InterruptedException {
      //  robot = new RobotClass(hardwareMap,telemetry,this);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, this, telemetry);
//        drive.openCVInnitShenanigans("blue");
//        FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition freightLocation = null;
//        freightLocation = drive.analyze();

        Pose2d startPose = new Pose2d(new Vector2d(-41.5, -63),Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        waitForStart();
//
//        long startTime = new Date().getTime();
//        long time = 0;
//
//        while (time < 1000 && opModeIsActive()) {
//            time = new Date().getTime() - startTime;
//            freightLocation = robot.analyze();
//
//            telemetry.addData("Position", freightLocation);
//            telemetry.update();
//        }

        if (isStopRequested()) return;

//      Deposit initial freight
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
             .lineToLinearHeading(new Pose2d(new Vector2d(-28, -40), Math.toRadians(45)))
                .build();
        drive.followTrajectorySequence(trajSeq);

//        Hey, Wayne, I'd appreciate it if you could fix yo' stuff.
//        drive.distanceSensorStuff();
       // robot.distanceSensorStuff();
        drive.dumpFreightMiddle();

//        To spin duck
             Pose2d position= drive.getLocalizer().getPoseEstimate();
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(position)
                .lineToLinearHeading(new Pose2d(new Vector2d(-58.25, -59.25), Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(trajSeq2);

        drive.jevilTurnCarousel(.5,5);
       // robot.jevilTurnCarousel(.5,5);

//        To pick up duck
        Pose2d position2= drive.getLocalizer().getPoseEstimate();
        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(position2)
                .lineToLinearHeading(new Pose2d(new Vector2d(-52, -60), Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(trajSeq3);

        drive.getCube();

        Pose2d position3= drive.getLocalizer().getPoseEstimate();
        TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(position3)
                .lineToLinearHeading(new Pose2d(new Vector2d(-28, -40), Math.toRadians(45)))
                .build();
        drive.followTrajectorySequence(trajSeq4);

//        drive.distanceSensorStuff();
      //  robot.distanceSensorStuff();

        Pose2d position4= drive.getLocalizer().getPoseEstimate();
        TrajectorySequence trajSeq5 = drive.trajectorySequenceBuilder(position4)
                .lineToLinearHeading(new Pose2d(new Vector2d(-61, -34), Math.toRadians(0)))
                .build();
        drive.followTrajectorySequence(trajSeq5);
    }



}
