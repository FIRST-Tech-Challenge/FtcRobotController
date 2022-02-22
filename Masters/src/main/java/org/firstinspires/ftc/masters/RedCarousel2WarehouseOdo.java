package org.firstinspires.ftc.masters;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.drive.DriveConstants;
import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;

import java.util.Date;

@Autonomous(name = "Red - Carousel 2 Warehouse (STATE)", group = "competition")
public class RedCarousel2WarehouseOdo extends LinearOpMode {

    final int SERVO_DROP_PAUSE=900;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, this, telemetry);

        drive.openCVInnitShenanigans("red");
        MultipleCameraCV.ShippingElementDeterminationPipeline.FreightPosition freightLocation = drive.analyze();

        Pose2d startPose = new Pose2d(new Vector2d(-35, -63), Math.toRadians(90));

        drive.setPoseEstimate(startPose);
        TrajectorySequence fromHubToWaitPos = drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())
                .lineToSplineHeading(new Pose2d(new Vector2d(5, -60), Math.toRadians(180)))
                .build();
        TrajectorySequence fromWaitPosToWarehouse = drive.trajectorySequenceBuilder(fromHubToWaitPos.end())
                .splineToLinearHeading(new Pose2d( new Vector2d(48, -66), Math.toRadians(180)), Math.toRadians(2))
                .build();
        drive.linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_LIFT);

        waitForStart();

        ElapsedTime elapsedTime = new ElapsedTime();

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 200 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            freightLocation = drive.analyze();

            telemetry.addData("Position", freightLocation);
            telemetry.update();
        }
        switch (freightLocation) {
            case LEFT:
                drive.linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_LOW);
                break;
            case MIDDLE:
                drive.linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_MIDDLE);
                break;
            case RIGHT:
                drive.linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_TOP);
                break;
            default:
                drive.linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_TOP);
        }
        drive.linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.linearSlideMotor.setPower(.8);

        if (isStopRequested()) return;

//      Deposit initial freight
        Pose2d hubPosition = new Pose2d(new Vector2d(-23, -38), Math.toRadians(45));
        TrajectorySequence toHub = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(hubPosition)
                .build();
        drive.followTrajectorySequence(toHub);

        drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_DROP);
        drive.pause(SERVO_DROP_PAUSE);
        drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);
        if (freightLocation== MultipleCameraCV.ShippingElementDeterminationPipeline.FreightPosition.LEFT){
            drive.pause(300);
        }
        drive.stopShippingElementCamera();
        drive.retract();

//        To spin duck
        Pose2d position = drive.getLocalizer().getPoseEstimate();

        TrajectorySequence toCarousel = drive.trajectorySequenceBuilder(position)
                .lineToLinearHeading(new Pose2d(new Vector2d(-61.5, -56.5), Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(toCarousel);

        drive.intakeMotor.setPower(0.8);
        drive.jevilTurnCarousel(.5, 4); //can we go faster?



        TrajectorySequence getOffCarousel = drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())
                .strafeTo(new Vector2d(-59.5, -50), SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        drive.followTrajectorySequence(getOffCarousel);


        drive.findDuckRed();

//        TrajectorySequence acquireDuck = drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())
//                .lineTo(new Vector2d(-55,-64))
//                .build();
//        drive.followTrajectorySequence(acquireDuck);

//        boolean hasDuck = drive

        drive.CV.duckWebcam.stopStreaming();

        drive.pause(250);
        drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_LIFT);
        drive.pause(200);
        drive.linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_TOP);
        drive.linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.linearSlideMotor.setPower(.8);

        position = drive.getLocalizer().getPoseEstimate();
        TrajectorySequence trajSeq6 = drive.trajectorySequenceBuilder(position)
                .lineToLinearHeading(hubPosition)
                .build();
        drive.followTrajectorySequence(trajSeq6);

        drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_DROP);
        drive.pause(SERVO_DROP_PAUSE);
        drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);
        if (freightLocation== MultipleCameraCV.ShippingElementDeterminationPipeline.FreightPosition.LEFT){
            drive.pause(300);
        }
        drive.retract();

//        This is going to red box
//        position = drive.getLocalizer().getPoseEstimate();
//        Pose2d parkPosition = new Pose2d(new Vector2d(-62, -35), Math.toRadians(0));
//        TrajectorySequence trajSeq7 = drive.trajectorySequenceBuilder(position)
//                .lineToLinearHeading(parkPosition)
//                .build();
//        drive.followTrajectorySequence(trajSeq7);

        drive.followTrajectorySequence(fromHubToWaitPos);
        double seconds = elapsedTime.seconds();
        while (seconds<26) {
            seconds = elapsedTime.seconds();
        }
        drive.followTrajectorySequence(fromWaitPosToWarehouse);
        drive.getCube();

    }


}