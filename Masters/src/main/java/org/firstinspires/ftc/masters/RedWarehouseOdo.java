package org.firstinspires.ftc.masters;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.masters.drive.DriveConstants;
import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;

import java.util.Date;

@Autonomous(name="park city red warehouse ")
public class RedWarehouseOdo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
      //  robot = new RobotClass(hardwareMap,telemetry,this);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, this, telemetry);

        drive.openCVInnitShenanigans("red");
        FreightFrenzyComputerVisionShippingElementReversion.SkystoneDeterminationPipeline.FreightPosition freightLocation = null;
        freightLocation = drive.analyze();

        Pose2d startPose = new Pose2d(new Vector2d(13.5, -63),Math.toRadians(90));

        drive.setPoseEstimate(startPose);
        TrajectorySequence fromStartToHub = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(()->{
                    drive.linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_TOP);
                    drive.linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.linearSlideMotor.setPower(.9);
                })
                .strafeTo(new Vector2d(-15, -44))
                .build();
        TrajectorySequence fromHubToWarehouse = drive.trajectorySequenceBuilder(fromStartToHub.end())
                .addDisplacementMarker(()->{
                    drive.linearSlideMotor.setTargetPosition(0);
                    drive.linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.linearSlideMotor.setPower(.4);
                })
                .lineToSplineHeading(new Pose2d(new Vector2d(5, -60), Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d( new Vector2d(48, -66), Math.toRadians(180)), Math.toRadians(0))
//                .lineToSplineHeading(new Pose2d(new Vector2d(8.5, -66), Math.toRadians(180)))
//                .lineTo(new Vector2d(48, -66))
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
        drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_DROP);
        drive.pause(1000);
        drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);

//        switch (freightLocation) {
//            case LEFT:
//                drive.dumpFreightBottom();
//                break;
//            case MIDDLE:
//                drive.dumpFreightMiddle();
//                break;
//            case RIGHT:
//                drive.dumpFreightTop();
//                break;
//            default:
//                drive.dumpFreightTop();
//        }

        drive.followTrajectorySequence(fromHubToWarehouse);

        //pick up cube
        drive.getCube();

        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())

                .lineTo(new Vector2d(15, -66))
                .addDisplacementMarker(()->{
                    drive.linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_TOP);
                    drive.linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.linearSlideMotor.setPower(.9);
                })
                .splineToSplineHeading(new Pose2d(-11, -48, Math.toRadians(90)), Math.toRadians(90))
                .build();
        drive.followTrajectorySequence(trajSeq3);

        drive.dumpFreightTop();

        drive.followTrajectorySequence(fromHubToWarehouse);
//
//
//        TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(new Vector2d(-10, -63), Math.toRadians(90)))
//                .build();
//        drive.followTrajectorySequence(trajSeq4);
//
//
//        position= drive.getLocalizer().getPoseEstimate();
//        TrajectorySequence trajSeq5 = drive.trajectorySequenceBuilder(position)
//                .lineToLinearHeading(new Pose2d(new Vector2d(-15, -44), Math.toRadians(90)))
//                .build();
//        drive.followTrajectorySequence(trajSeq5);
//
////        Hey, Wayne, I'd appreciate it if you could fix yo' stuff.
////        drive.distanceSensorStuff();
//        // robot.distanceSensorStuff();
//        drive.dumpFreightTop();
//        position= drive.getLocalizer().getPoseEstimate();
//        TrajectorySequence trajSeq6 = drive.trajectorySequenceBuilder(position)
//                .lineToLinearHeading(new Pose2d(new Vector2d(8.5, -70), Math.toRadians(180)))
//                .build();
//        drive.followTrajectorySequence(trajSeq6);
//
//
//        position= drive.getLocalizer().getPoseEstimate();
//        TrajectorySequence trajSeq7 = drive.trajectorySequenceBuilder(position)
//                .lineToLinearHeading(new Pose2d(new Vector2d(48, -70), Math.toRadians(180)))
//                .build();
//        drive.followTrajectorySequence(trajSeq7);

//        To spin duck
//             Pose2d position= drive.getLocalizer().getPoseEstimate();
//        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(new Vector2d(-59.25, -59.25), Math.toRadians(90)))
//                .build();
//        drive.followTrajectorySequence(trajSeq3);
//
//        drive.jevilTurnCarousel(.3,6 );
//        drive.intakeMotor.setPower(.8);
//
////
//        position= drive.getLocalizer().getPoseEstimate();
//        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(position)
//                .lineToLinearHeading(new Pose2d(new Vector2d(-72+15, -72+7), Math.toRadians(110)), SampleMecanumDrive.getVelocityConstraint(10,DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//        drive.followTrajectorySequence(trajSeq3);
//
//        position= drive.getLocalizer().getPoseEstimate();
//        TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(position)
//                .lineToLinearHeading(new Pose2d(new Vector2d(-29, -72+7), Math.toRadians(110)))
//                .build();
//        drive.followTrajectorySequence(trajSeq4);
//
////
//        position= drive.getLocalizer().getPoseEstimate();
//        TrajectorySequence trajSeq5 = drive.trajectorySequenceBuilder(position)
//                .lineToLinearHeading(new Pose2d(new Vector2d(-29, -65), Math.toRadians(110)))
//                .build();
//        drive.followTrajectorySequence(trajSeq5);
//
//
//        position= drive.getLocalizer().getPoseEstimate();
//        TrajectorySequence trajSeq6 = drive.trajectorySequenceBuilder(position)
//                .lineToLinearHeading(new Pose2d(new Vector2d(-29, -37), Math.toRadians(90)))
//                .build();
//        drive.followTrajectorySequence(trajSeq6);
//
//        drive.dumpFreightTop();
//
//        position= drive.getLocalizer().getPoseEstimate();
//        TrajectorySequence trajSeq7 = drive.trajectorySequenceBuilder(position)
//                .lineToLinearHeading(new Pose2d(new Vector2d(-61, -34), Math.toRadians(0)))
//                .build();
//        drive.followTrajectorySequence(trajSeq7);
//
//
//        Pose2d position5= drive.getLocalizer().getPoseEstimate();
//        TrajectorySequence trajSeq8 = drive.trajectorySequenceBuilder(position5)
//                .lineToLinearHeading(new Pose2d(new Vector2d(-61, -34), Math.toRadians(0)))
//                .build();
//        drive.followTrajectorySequence(trajSeq8);
    }



}
