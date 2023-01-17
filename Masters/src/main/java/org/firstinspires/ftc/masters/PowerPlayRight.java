package org.firstinspires.ftc.masters;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;

import java.util.Date;

@Autonomous(name = "Power Play Right")
public class PowerPlayRight extends LinearOpMode {

    @Override
    public void runOpMode() {

        PowerPlayComputerVisionPipelines CV = new PowerPlayComputerVisionPipelines(hardwareMap, telemetry);
        PowerPlayComputerVisionPipelines.SleevePipeline.SleeveColor sleeveColor = null;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(new Vector2d(37.5, -64.25), Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        drive.closeClaw();

        waitForStart();
        drive.closeClaw();

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 200 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            sleeveColor = CV.sleevePipeline.color;

            telemetry.addData("Position", sleeveColor);
            telemetry.update();
        }

        drive.closeClaw();

        drive.setArmServoMiddle();

//        drive.liftTop();

        TrajectorySequence startTo270Pole = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d( new Vector2d(14,-60), Math.toRadians(90)), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(new Vector2d(10.75,-35.5),Math.toRadians(135)))
                .build();
        drive.followTrajectorySequence(startTo270Pole);


        //use vision to align

        //drop cone
//        drive.liftMiddle();
//        sleep(1000);
//        drive.openClaw();
//        sleep(300);
//        drive.liftTop();

//        TrajectorySequence back = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .back(2)
//                .build();
//        drive.followTrajectorySequence(back);
//        drive.turn(Math.toRadians(135));

////        drive.setArmServoTop();
////        drive.liftDown();
////        while (this.opModeIsActive() && (drive.linearSlide.getCurrentPosition()>200|| drive.frontSlide.getCurrentPosition()>200)){
////
////        }
////        drive.setArmServoBottom();
//
        TrajectorySequence secondCone = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .turn(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d( new Vector2d(20,-14), Math.toRadians(0)),Math.toRadians(0))
                 .build();
        drive.followTrajectorySequence(secondCone);
//
////        drive.openClaw();
//
        TrajectorySequence score = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(new Pose2d(new Vector2d(32.5,-11.5),Math.toRadians(315)),Math.toRadians(155))
                .build();
//        drive.followTrajectorySequence(score);

//
        TrajectorySequence newCone = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(new Vector2d(59,-14),Math.toRadians(0)))
                .build();
        drive.followTrajectorySequence(newCone);
//
        drive.followTrajectorySequence(score);
//
//        drive.followTrajectorySequence(newCone);
//
//        drive.followTrajectorySequence(score);
//
//        drive.followTrajectorySequence(newCone);
//
//        drive.followTrajectorySequence(score);
//
//        drive.turn(90);
//
//        switch (sleeveColor) {
//            case GRAY:
//                //Parking 1
//                TrajectorySequence park1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                        .strafeTo(new Vector2d(64, -12))
//                        .build();
//                drive.followTrajectorySequence(park1);
//                break;
//            case RED:
//                //Parking 2
//                TrajectorySequence park2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                        .strafeTo(new Vector2d(38, -12))
//                        .build();
//                drive.followTrajectorySequence(park2);
//                break;
//            case GREEN:
//                //Parking 3
//                break;
//            case INDETERMINATE:
//                break;
//
//        }

    }

}
