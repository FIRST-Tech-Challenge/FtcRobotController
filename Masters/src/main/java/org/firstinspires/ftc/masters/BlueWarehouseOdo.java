package org.firstinspires.ftc.masters;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;

import java.util.Date;

@Autonomous(name="park city blue warehouse ")
public class BlueWarehouseOdo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
      //  robot = new RobotClass(hardwareMap,telemetry,this);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, this, telemetry);

        drive.openCVInnitShenanigans("blue");
        FreightFrenzyComputerVisionRedHub.SkystoneDeterminationPipeline.FreightPosition freightLocation = null;
        freightLocation = drive.analyze();

        Pose2d startPose = new Pose2d(new Vector2d(13.5, 63),Math.toRadians(270));

        drive.setPoseEstimate(startPose);
        TrajectorySequence fromStartToHub = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(-15, 44))
                .build();

        TrajectorySequence fromHubToWarehouse = drive.trajectorySequenceBuilder(fromStartToHub.end())
                .lineToSplineHeading(new Pose2d(new Vector2d(5, 60), Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d( new Vector2d(48, 66), Math.toRadians(180)), Math.toRadians(0))
                .build();



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

        drive.followTrajectorySequence(fromStartToHub);

        switch (freightLocation) {
            case LEFT:
                drive.dumpFreightBottom();
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

        drive.followTrajectorySequence(fromHubToWarehouse);

        //pick up cube



        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())
                .lineTo(new Vector2d(15, 66))
                .splineToSplineHeading(new Pose2d(-11, 48, Math.toRadians(270)), Math.toRadians(270))
                .build();
        drive.followTrajectorySequence(trajSeq3);

        drive.dumpFreightTop();

        drive.followTrajectorySequence(fromHubToWarehouse);

    }



}
