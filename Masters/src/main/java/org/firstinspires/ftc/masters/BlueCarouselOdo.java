package org.firstinspires.ftc.masters;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;

import java.util.Date;

@Autonomous(name = "Blue - Carousel (STATE)", group = "competition")
public class BlueCarouselOdo extends LinearOpMode {

    final int SERVO_DROP_PAUSE=900;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, this, telemetry);

        drive.openCVInnitShenanigans("red");
        MultipleCameraCV.ShippingElementDeterminationPipeline.FreightPosition freightLocation = null;
        freightLocation = drive.analyze();

        Pose2d startPose = new Pose2d(new Vector2d(-35, 63), Math.toRadians(270));

        drive.setPoseEstimate(startPose);
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
        Pose2d hubPosition = new Pose2d(new Vector2d(-11.5, 43), Math.toRadians(270));
        TrajectorySequence toHub = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(hubPosition)
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
                .lineToLinearHeading(new Pose2d( new Vector2d(-58.5, 58.5), Math.toRadians(0)))
                .build();
        drive.followTrajectorySequence(toCarousel);

        drive.intakeMotor.setPower(0.8);
        drive.jevilTurnCarousel(-.4, 4);

        TrajectorySequence leaveCarousel = drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())
                .lineTo(new Vector2d(-55, 47))
                .turn(Math.toRadians(-90))
                .build();
        drive.followTrajectorySequence(leaveCarousel);

       // drive.turn(Math.toRadians(-90));

//        TrajectorySequence AcquireDuck = drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())
//                .lineTo(new Vector2d(-55, 63))
//                .build();
//        drive.followTrajectorySequence(AcquireDuck);

        drive.findDuckBlue();


        position = drive.getLocalizer().getPoseEstimate();

        drive.pause(200);
        drive.intakeMotor.setPower(-0.8);
        drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_LIFT);
        drive.linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_TOP);
        drive.linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.linearSlideMotor.setPower(.8);
        drive.intakeMotor.setPower(0);

        TrajectorySequence depositDuck = drive.trajectorySequenceBuilder(position)
                .lineTo(new Vector2d(-50, 55))
                .splineToLinearHeading(new Pose2d(-11, 43, Math.toRadians(270)), Math.toRadians(270))
                .build();
        drive.followTrajectorySequence(depositDuck);

        drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_DROP);
        drive.pause(SERVO_DROP_PAUSE);
        drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);

        drive.retract();

        position = drive.getLocalizer().getPoseEstimate();
        TrajectorySequence trajSeq7 = drive.trajectorySequenceBuilder(position)
//                .strafeRight(20)
                .lineTo(new Vector2d(-32, 43))
                .splineToLinearHeading (new Pose2d(new Vector2d(-62, 35),Math.toRadians(270)), Math.toRadians(180))
                .build();
        drive.followTrajectorySequence(trajSeq7);

    }


}


// MeepMeepTesting Code &, more importantly, Vectors and Poses

///drive.trajectorySequenceBuilder(new Pose2d(-35, 63, Math.toRadians(270)))
//        .lineToSplineHeading(new Pose2d(new Vector2d(-12.5, 42), Math.toRadians(270)))
//        .lineToLinearHeading(new Pose2d( new Vector2d(-60, 60), Math.toRadians(0)))
//        .lineTo(new Vector2d(-55, 55))
//        .lineToLinearHeading(new Pose2d(-55, 54, Math.toRadians(270)))
//        .lineTo(new Vector2d(-55, 63))
//        .lineTo(new Vector2d(-50, 55))
//        .splineToLinearHeading(new Pose2d(-12.5, 42, Math.toRadians(270)), Math.toRadians(270))
//        .strafeRight(20)
//        //.lineTo(new Vector2d(-22, 44))
//        .splineToLinearHeading (new Pose2d(new Vector2d(-62, 35),Math.toRadians(270)), Math.toRadians(180))
//        .build() );