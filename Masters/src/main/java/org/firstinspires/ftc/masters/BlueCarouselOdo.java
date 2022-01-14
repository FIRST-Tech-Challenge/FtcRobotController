package org.firstinspires.ftc.masters;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.drive.DriveConstants;
import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;

import java.util.Date;

@Autonomous(name = "Blue carousel odometry", group = "competition")
public class BlueCarouselOdo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, this, telemetry);

        drive.openCVInnitShenanigans("red");
        FreightFrenzyComputerVisionShippingElementReversion.SkystoneDeterminationPipeline.FreightPosition freightLocation = null;
        freightLocation = drive.analyze();

        Pose2d startPose = new Pose2d(new Vector2d(-35, 63), Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        waitForStart();

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 200 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            freightLocation = drive.analyze();

            telemetry.addData("Position", freightLocation);
            telemetry.update();
        }

        if (isStopRequested()) return;

//      Deposit initial freight
        Pose2d hubPosition = new Pose2d(new Vector2d(-12.5, 42), Math.toRadians(270));
        TrajectorySequence toHub = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(hubPosition)
                .build();
        drive.followTrajectorySequence(toHub);

        switch (freightLocation) {
            case LEFT:
                drive.dumpFreightTop();
                break;
            case MIDDLE:
                drive.dumpFreightMiddle();
                break;
            case RIGHT:
                drive.dumpFreightTop();
                break;
            default:
                drive.dumpFreightTop();
        }

//        To spin duck
        Pose2d position = drive.getLocalizer().getPoseEstimate();

        TrajectorySequence toCarousel = drive.trajectorySequenceBuilder(position)
                .lineToLinearHeading(new Pose2d( new Vector2d(-58.5, 58.5), Math.toRadians(0)))
                .build();
        drive.followTrajectorySequence(toCarousel);

        drive.intakeMotor.setPower(0.8);
        drive.jevilTurnCarousel(.4, 5);

        TrajectorySequence leaveCarouselAndAcquireDuck = drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())
                .lineTo(new Vector2d(-55, 55))
                .turn(270)
                .lineTo(new Vector2d(-55, 63))
                .build();
        drive.followTrajectorySequence(leaveCarouselAndAcquireDuck);

        position = drive.getLocalizer().getPoseEstimate();

        TrajectorySequence depositDuck = drive.trajectorySequenceBuilder(position)
                .lineTo(new Vector2d(-50, 55))
                .splineToLinearHeading(new Pose2d(-12.5, 42, Math.toRadians(270)), Math.toRadians(270))
                .build();
        drive.followTrajectorySequence(depositDuck);

        drive.dumpFreightTop();

        position = drive.getLocalizer().getPoseEstimate();
        TrajectorySequence trajSeq7 = drive.trajectorySequenceBuilder(position)
                .lineTo(new Vector2d(-22, 44))
                .lineTo(new Vector2d(-62, 35))
                .build();
        drive.followTrajectorySequence(trajSeq7);

    }


}


// MeepMeepTesting Code &, more importantly, Vectors and Poses

//RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//        .setConstraints(45, 60, Math.toRadians(60), Math.toRadians(60), 16.4)
//        .followTrajectorySequence(drive ->
//                drive.trajectorySequenceBuilder(new Pose2d(-34, 63, Math.toRadians(270)))
//                        .lineToSplineHeading(new Pose2d(new Vector2d(-12.5, 42), Math.toRadians(270)))
//                        .lineToLinearHeading(new Pose2d( new Vector2d(-60, 60), Math.toRadians(0)))
//                        .lineTo(new Vector2d(-55, 55))
//                        .lineToLinearHeading(new Pose2d(-55, 54, Math.toRadians(270)))
//                        .lineTo(new Vector2d(-55, 63))
//                        .lineTo(new Vector2d(-50, 55))
//                        .splineToLinearHeading(new Pose2d(-12.5, 42, Math.toRadians(270)), Math.toRadians(270))
//                        .lineTo(new Vector2d(-22, 44))
//                        .lineTo(new Vector2d(-61, 34))
//                        .build()
//        );