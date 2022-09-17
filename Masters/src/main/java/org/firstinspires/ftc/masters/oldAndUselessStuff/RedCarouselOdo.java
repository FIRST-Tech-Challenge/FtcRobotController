package org.firstinspires.ftc.masters.oldAndUselessStuff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.FreightFrenzyConstants;
import org.firstinspires.ftc.masters.MultipleCameraCV;
import org.firstinspires.ftc.masters.drive.DriveConstants;
import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;

import java.util.Date;

@Autonomous(name = "Red - Carousel (STATE)", group = "competition")
public class RedCarouselOdo extends LinearOpMode {

    final int SERVO_DROP_PAUSE = 900;
    Pose2d position;
    SampleMecanumDrive drive;
    ElapsedTime elapsedTime;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap, this, telemetry);

        drive.openCVInnitShenanigans();
        MultipleCameraCV.ShippingElementDeterminationPipeline.ElementPosition freightLocation = drive.analyze();

        Pose2d startPose = new Pose2d(new Vector2d(-35, -63), Math.toRadians(90));

        drive.setPoseEstimate(startPose);
        drive.linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.linearSlideServo.setPosition(org.firstinspires.ftc.masters.FreightFrenzyConstants.DUMP_SERVO_LIFT);

        waitForStart();
//        Mecha Knight Changes: ALERT
//        drive.pause(1000);

        elapsedTime = new ElapsedTime();

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
                drive.linearSlideMotor.setTargetPosition(org.firstinspires.ftc.masters.FreightFrenzyConstants.SLIDE_LOW);
                break;
            case MIDDLE:
                drive.linearSlideMotor.setTargetPosition(org.firstinspires.ftc.masters.FreightFrenzyConstants.SLIDE_MIDDLE);
                break;
            case RIGHT:
                drive.linearSlideMotor.setTargetPosition(org.firstinspires.ftc.masters.FreightFrenzyConstants.SLIDE_TOP);
                break;
            default:
                drive.linearSlideMotor.setTargetPosition(org.firstinspires.ftc.masters.FreightFrenzyConstants.SLIDE_TOP);
        }
        drive.linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.linearSlideMotor.setPower(.8);

        if (isStopRequested()) return;

//      Deposit initial freight
        Pose2d hubPosition = new Pose2d(new Vector2d(-23, -38), Math.toRadians(47));
        TrajectorySequence toHubHigh = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(hubPosition)
                .build();
        TrajectorySequence toHubLow = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(new Vector2d(-22, -36.5), Math.toRadians(47)))
                .build();

        switch (freightLocation) {
            case LEFT:
                drive.followTrajectorySequence(toHubLow);
                break;
            case MIDDLE:
                drive.followTrajectorySequence(toHubHigh);
                break;
            case RIGHT:
                drive.followTrajectorySequence(toHubHigh);
                break;
        }
        drive.linearSlideServo.setPosition(org.firstinspires.ftc.masters.FreightFrenzyConstants.DUMP_SERVO_DROP);
        drive.pause(SERVO_DROP_PAUSE);
        drive.linearSlideServo.setPosition(org.firstinspires.ftc.masters.FreightFrenzyConstants.DUMP_SERVO_BOTTOM);
        if (freightLocation == MultipleCameraCV.ShippingElementDeterminationPipeline.ElementPosition.LEFT) {
            drive.pause(300);
        }
        drive.stopShippingElementCamera();
        drive.retract();

//        To spin duck
        position = drive.getLocalizer().getPoseEstimate();

        TrajectorySequence toCarousel = drive.trajectorySequenceBuilder(position)
                .lineToLinearHeading(new Pose2d(new Vector2d(-61.5, -56.5), Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(toCarousel);

        drive.intakeMotor.setPower(1);
        drive.jevilTurnRedCarousel(3); //can we go faster?


        TrajectorySequence getOffCarousel = drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())
                .strafeTo(new Vector2d(-59.5, -50), SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        drive.followTrajectorySequence(getOffCarousel);


        drive.findDuckRed();


        drive.CV.duckWebcam.stopStreaming();

        drive.pause(350);
        drive.linearSlideServo.setPosition(org.firstinspires.ftc.masters.FreightFrenzyConstants.DUMP_SERVO_LIFT);
        drive.pause(250);
        drive.intakeMotor.setPower(0);
        drive.linearSlideMotor.setTargetPosition(org.firstinspires.ftc.masters.FreightFrenzyConstants.SLIDE_TOP);
        drive.linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.linearSlideMotor.setPower(.7);

        position = drive.getLocalizer().getPoseEstimate();
        position.minus(new Pose2d(12,0,0));
        TrajectorySequence trajSeq6 = drive.trajectorySequenceBuilder(position)
                .lineToLinearHeading(hubPosition)
                .build();
        drive.followTrajectorySequence(trajSeq6);

        drive.linearSlideServo.setPosition(org.firstinspires.ftc.masters.FreightFrenzyConstants.DUMP_SERVO_DROP);
        drive.pause(SERVO_DROP_PAUSE);
        drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);
        if (freightLocation == MultipleCameraCV.ShippingElementDeterminationPipeline.ElementPosition.LEFT) {
            drive.pause(300);
        }
        drive.retract();


        gotToPark();

    }

    protected void gotToPark() {
        position = drive.getLocalizer().getPoseEstimate();
        Pose2d parkPosition = new Pose2d(new Vector2d(-62, -35), Math.toRadians(0));
        TrajectorySequence trajSeq7 = drive.trajectorySequenceBuilder(position)
                .lineToLinearHeading(parkPosition)
                .build();
        drive.followTrajectorySequence(trajSeq7);
    }


}
