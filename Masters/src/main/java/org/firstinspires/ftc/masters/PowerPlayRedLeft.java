package org.firstinspires.ftc.masters;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;

import java.util.Date;

@Autonomous(name = "Power Play Red Left")
public class PowerPlayRedLeft extends LinearOpMode {


    Pose2d westPoleDeposit = new Pose2d(new Vector2d(-12,-12),Math.toRadians(135));
    Pose2d coneStack = new Pose2d(new Vector2d(-60,-12),Math.toRadians(180));

    @Override
    public void runOpMode() {

        PowerPlayComputerVisionPipelines CV = new PowerPlayComputerVisionPipelines(hardwareMap, telemetry);
        PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor sleeveColor = null;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap,  telemetry);
        Pose2d startPose = new Pose2d(new Vector2d(-36, -60), Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        waitForStart();

        ElapsedTime elapsedTime = new ElapsedTime();

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 200 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            sleeveColor = CV.sleevePipeline.color;

            telemetry.addData("Position", sleeveColor);
            telemetry.update();
        }

//        drive.liftTop();
        TrajectorySequence startTo270Pole = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d( new Vector2d(- 9,-50), Math.toRadians(90)), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(new Vector2d(-9,-36),Math.toRadians(45)))
//                .lineToLinearHeading(westPoleDeposit)
                .build();
        drive.followTrajectorySequence(startTo270Pole);

        //use vision to align
        //drop cone
        //drive.openClaw();

        switch (sleeveColor) {
            case RED:
                //Parking 2
            case GRAY:
                //Parking 1
            case GREEN:
                //Parking 3
            case INDETERMINATE:

        }



//        TrajectorySequence toConeStack = drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())
//                .lineToLinearHeading(coneStack)
//                .build();
//        drive.followTrajectorySequence(toConeStack);
//
//
//        TrajectorySequence toWestPole = drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())
//                .lineToLinearHeading(westPoleDeposit)
//                .build();
//        drive.followTrajectorySequence(toWestPole);
//

        //park in the correct spot
//        drive.followTrajectorySequence(toConeStack);

        //put lift down

    }

}
