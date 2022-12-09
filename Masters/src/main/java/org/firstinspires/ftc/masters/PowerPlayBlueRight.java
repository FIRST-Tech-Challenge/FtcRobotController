package org.firstinspires.ftc.masters;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;

import java.util.Date;

@Autonomous(name = "Power Play Blue Right")
public class PowerPlayBlueRight extends LinearOpMode{

    @Override
    public void runOpMode() {

        PowerPlayComputerVisionPipelines CV = new PowerPlayComputerVisionPipelines(hardwareMap, telemetry);
        PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor sleeveColor = null;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(new Vector2d(-36, 60), Math.toRadians(-90));
        drive.setPoseEstimate(startPose);
        drive.closeClaw();
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

        drive.setArmServoMiddle();

        drive.liftTop();

        TrajectorySequence startTo270Pole = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d( new Vector2d( -9,50), Math.toRadians(-90)), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(new Vector2d(-8,27),Math.toRadians(-135)))
//                .lineToLinearHeading(westPoleDeposit)
                .build();
        drive.followTrajectorySequence(startTo270Pole);

        //use vision to align
        //drop cone
        drive.openClaw();
        sleep(300);

        TrajectorySequence back = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(2)
                .build();
        drive.followTrajectorySequence(back);
        drive.setArmServoBottom();
        drive.turn(Math.toRadians(-45));

        //When parking
        switch (sleeveColor) {
            case RED:
                TrajectorySequence park2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .strafeTo(new Vector2d(-36, 34))
                        .build();
                drive.followTrajectorySequence(park2);
                //Parking 2
            case GRAY:

            case GREEN:
                TrajectorySequence park1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .strafeTo(new Vector2d(-62, 34))
                        .build();
                drive.followTrajectorySequence(park1);
                //Parking 3
            case INDETERMINATE:

        }

    }

}