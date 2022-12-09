//package org.firstinspires.ftc.masters.oldAndUselessStuff;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.masters.FreightFrenzyConstants;
//import org.firstinspires.ftc.masters.MultipleCameraCV;
//import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;
//
//import java.util.Date;
//
//@Autonomous(name="Blue - Warehouse (STATE)", group = "competition")
//public class BlueWarehouseOdo extends LinearOpMode {
//
//    final int SERVO_DROP_PAUSE=900;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//      //  robot = new RobotClass(hardwareMap,telemetry,this);
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, this, telemetry);
//
//        drive.openCVInnitShenanigans();
//        MultipleCameraCV.ShippingElementDeterminationPipeline.ElementPosition freightLocation = null;
//        freightLocation = drive.analyze();
//
//        Pose2d startPose = new Pose2d(new Vector2d(13.5, 63),Math.toRadians(270));
//
//        drive.setPoseEstimate(startPose);
//        TrajectorySequence fromStartToHubHigh = drive.trajectorySequenceBuilder(startPose)
//                .strafeTo(new Vector2d(-12, 43))
//                .build();
//        TrajectorySequence fromStartToHubLow = drive.trajectorySequenceBuilder(startPose)
//                .strafeTo(new Vector2d(-12, 41.7))
//                .build();
//
//        TrajectorySequence fromHubToWarehouse = drive.trajectorySequenceBuilder(fromStartToHubHigh.end())
//                .lineToSplineHeading(new Pose2d(new Vector2d(5, 60), Math.toRadians(180)))
//                .splineToLinearHeading(new Pose2d( new Vector2d(48, 66), Math.toRadians(180)), Math.toRadians(0))
//                .build();
//        TrajectorySequence roomyWarehouse = drive.trajectorySequenceBuilder(fromHubToWarehouse.end())
//                .strafeLeft(24)
//                .build();
//        drive.  linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_LIFT);
//        waitForStart();
//
//        drive.stopDuckCamera();
//        long startTime = new Date().getTime();
//        long time = 0;
//
//        while (time < 200 && opModeIsActive()) {
//            time = new Date().getTime() - startTime;
//            freightLocation = drive.analyze();
//
//            telemetry.addData("Position", freightLocation);
//            telemetry.update();
//        }
//
////        Deposit initial freight
//        switch (freightLocation) {
//            case LEFT:
//                drive.linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_LOW);
//                break;
//            case MIDDLE:
//                drive.linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_MIDDLE);
//                break;
//            case RIGHT:
//                drive.linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_TOP);
//                break;
//            default:
//                drive.linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_TOP);
//        }
//        drive.linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        drive.linearSlideMotor.setPower(.8);
//
//        if (isStopRequested()) return;
//
//        switch (freightLocation) {
//            case LEFT:
//                drive.followTrajectorySequence(fromStartToHubLow);
//                break;
//            case MIDDLE:
//                drive.followTrajectorySequence(fromStartToHubHigh);
//                break;
//            case RIGHT:
//                drive.followTrajectorySequence(fromStartToHubHigh);
//                break;
//        }
//        drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_DROP);
//        drive.pause(SERVO_DROP_PAUSE);
//        drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);
//        if (freightLocation== MultipleCameraCV.ShippingElementDeterminationPipeline.ElementPosition.LEFT){
//            drive.pause(300);
//        }
//        drive.retract();
//
//
////        Cycle
//        drive.followTrajectorySequence(fromHubToWarehouse);
//
//        //pick up cube
////        boolean gotCube= drive.getCube(1400);
////        if (!gotCube){
////            drive.forward(-.5, .3);
////            drive.strafeLeft(.5,.35);
////            gotCube = drive.getCube(1400);
////        }
////
////
////        if (gotCube) {
////            TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())
////                    //.addDisplacementMarker(()-> drive.intakeMotor.setPower(1))
////                    .lineTo(new Vector2d(15, 66))
////                    .addDisplacementMarker(() -> {
////                        drive.intakeMotor.setPower(1);
////                        drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_LIFT);
////                        drive.linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_TOP);
////                        drive.linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                        drive.linearSlideMotor.setPower(.8);
////                        drive.intakeMotor.setPower(0);
////                    })
////                    .splineToSplineHeading(new Pose2d(-10, 45, Math.toRadians(270)), Math.toRadians(270))
//////                    -14, 46
////                    .build();
////            drive.followTrajectorySequence(trajSeq3);
////            fromHubToWarehouse= drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())
////                    .lineToSplineHeading(new Pose2d(new Vector2d(5, 61), Math.toRadians(180)))
////                    .splineToLinearHeading(new Pose2d( new Vector2d(48, 68), Math.toRadians(180)), Math.toRadians(0))
////                    .build();
////            drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_DROP);
////            drive.pause(SERVO_DROP_PAUSE);
////            drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);
////            drive.retract();
////
////
////            drive.followTrajectorySequence(fromHubToWarehouse);
////            if (drive.getCube(2000)) {
////
////                trajSeq3 = drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())
////                        //.addDisplacementMarker(()-> drive.intakeMotor.setPower(1))
////                        .lineTo(new Vector2d(15, 66))
////                        .addDisplacementMarker(() -> {
////                            drive.intakeMotor.setPower(1);
////                            drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_LIFT);
////                            drive.linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_TOP);
////                            drive.linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                            drive.linearSlideMotor.setPower(.8);
////                            drive.intakeMotor.setPower(0);
////                        })
////                        .splineToSplineHeading(new Pose2d(-10, 46, Math.toRadians(270)), Math.toRadians(270))
////                        .build();
////                drive.followTrajectorySequence(trajSeq3);
////                fromHubToWarehouse= drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())
////                        .lineToSplineHeading(new Pose2d(new Vector2d(5, 62), Math.toRadians(180)))
////                        .splineToLinearHeading(new Pose2d( new Vector2d(48, 69), Math.toRadians(180)), Math.toRadians(0))
////                        .build();
////                drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_DROP);
////                drive.pause(SERVO_DROP_PAUSE);
////                drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);
////                drive.retract();
////
////                drive.followTrajectorySequence(fromHubToWarehouse);
////                drive.getCube(2000);
////            }
////        }
//
//        //drive.followTrajectorySequence(roomyWarehouse);
//
//    }
//
//
//
//}
