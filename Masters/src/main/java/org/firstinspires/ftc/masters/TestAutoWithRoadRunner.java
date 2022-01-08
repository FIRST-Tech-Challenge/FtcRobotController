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

    @Override
    public void runOpMode() throws InterruptedException {
      //  robot = new RobotClass(hardwareMap,telemetry,this);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, this, telemetry);

        drive.openCVInnitShenanigans("red");
        FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition freightLocation = null;
        freightLocation = drive.analyze();

        Pose2d startPose = new Pose2d(new Vector2d(-41.5, -63),Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        waitForStart();

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 1000 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            freightLocation = drive.analyze();

            telemetry.addData("Position", freightLocation);
            telemetry.update();
        }

        if (isStopRequested()) return;

//      Deposit initial freight
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
             .lineToLinearHeading(new Pose2d(new Vector2d(-29, -37), Math.toRadians(45)))
                .build();
        drive.followTrajectorySequence(trajSeq);

//        Hey, Wayne, I'd appreciate it if you could fix yo' stuff.
//        drive.distanceSensorStuff();
       // robot.distanceSensorStuff();
        drive.dumpFreightTop();

//        To spin duck
             Pose2d position= drive.getLocalizer().getPoseEstimate();
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(position)
                .lineToLinearHeading(new Pose2d(new Vector2d(-59.25, -59.25), Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(trajSeq2);

        drive.jevilTurnCarousel(.3,6 );
        drive.intakeMotor.setPower(.8);

//
        position= drive.getLocalizer().getPoseEstimate();
        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(position)
                .lineToLinearHeading(new Pose2d(new Vector2d(-72+15, -72+7), Math.toRadians(110)), SampleMecanumDrive.getVelocityConstraint(10,DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        drive.followTrajectorySequence(trajSeq3);

        position= drive.getLocalizer().getPoseEstimate();
        TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(position)
                .lineToLinearHeading(new Pose2d(new Vector2d(-29, -72+7), Math.toRadians(110)))
                .build();
        drive.followTrajectorySequence(trajSeq4);

//
        position= drive.getLocalizer().getPoseEstimate();
        TrajectorySequence trajSeq5 = drive.trajectorySequenceBuilder(position)
                .lineToLinearHeading(new Pose2d(new Vector2d(-29, -65), Math.toRadians(110)))
                .build();
        drive.followTrajectorySequence(trajSeq5);


        position= drive.getLocalizer().getPoseEstimate();
        TrajectorySequence trajSeq6 = drive.trajectorySequenceBuilder(position)
                .lineToLinearHeading(new Pose2d(new Vector2d(-29, -37), Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(trajSeq6);

        drive.dumpFreightTop();

        position= drive.getLocalizer().getPoseEstimate();
        TrajectorySequence trajSeq7 = drive.trajectorySequenceBuilder(position)
                .lineToLinearHeading(new Pose2d(new Vector2d(-61, -34), Math.toRadians(0)))
                .build();
        drive.followTrajectorySequence(trajSeq7);


        Pose2d position5= drive.getLocalizer().getPoseEstimate();
        TrajectorySequence trajSeq8 = drive.trajectorySequenceBuilder(position5)
                .lineToLinearHeading(new Pose2d(new Vector2d(-61, -34), Math.toRadians(0)))
                .build();
        drive.followTrajectorySequence(trajSeq8);
    }



}
