//package org.firstinspires.ftc.masters;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.masters.drive.DriveConstants;
//import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;
//
//import java.util.Date;
//
//@Disabled
////@Autonomous(name = "test auto with roadRunner")
//public class TestAutoWithRoadRunner extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        //  robot = new RobotClass(hardwareMap,telemetry,this);
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, this, telemetry);
//
//        drive.openCVInnitShenanigans("red");
//        TheAbsolutelyPositivelyWithoutAShadowOfADoubtFinalLastIterationOfFreightFrenzyCV.SkystoneDeterminationPipeline.FreightPosition freightLocation = null;
//        freightLocation = drive.analyze();
//
//        Pose2d startPose = new Pose2d(new Vector2d(-41.5, -63), Math.toRadians(90));
//
//        drive.setPoseEstimate(startPose);
//
//        waitForStart();
//
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
//        if (isStopRequested()) return;
//
////      Deposit initial freight
//        Pose2d hubPosition = new Pose2d(new Vector2d(-29, -37), Math.toRadians(45));
//        TrajectorySequence toHub = drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(hubPosition)
//                .build();
//        drive.followTrajectorySequence(toHub);
//
////        Hey, Wayne, I'd appreciate it if you could fix yo' stuff.
////        drive.distanceSensorStuff();
//        // robot.distanceSensorStuff();
//        switch (freightLocation) {
//            case LEFT:
//                drive.dumpFreightTop();
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
//
////        To spin duck
//        Pose2d position = drive.getLocalizer().getPoseEstimate();
//
//        TrajectorySequence toCarousel = drive.trajectorySequenceBuilder(position)
//                .lineToLinearHeading(new Pose2d(new Vector2d(-59.25, -59.25), Math.toRadians(90)))
//                .build();
//        drive.followTrajectorySequence(toCarousel);
//
//        drive.jevilTurnCarousel(.3, 6); //can we go faster?
//        drive.intakeMotor.setPower(.8);
//
////
//        position = drive.getLocalizer().getPoseEstimate();
//        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(position)
//                .lineToLinearHeading(new Pose2d(new Vector2d(-72 + 15, -72 + 7), Math.toRadians(110)), SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//        drive.followTrajectorySequence(trajSeq3);
//
//        position = drive.getLocalizer().getPoseEstimate();
//        TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(position)
//                .lineToLinearHeading(new Pose2d(new Vector2d(-29, -72 + 7), Math.toRadians(110)))
//                .build();
//        drive.followTrajectorySequence(trajSeq4);
//
////
//        position = drive.getLocalizer().getPoseEstimate();
//        TrajectorySequence trajSeq5 = drive.trajectorySequenceBuilder(position)
//                .lineToLinearHeading(new Pose2d(new Vector2d(-29, -65), Math.toRadians(110)))
//                .build();
//        drive.followTrajectorySequence(trajSeq5);
//
//
//        position = drive.getLocalizer().getPoseEstimate();
//        TrajectorySequence trajSeq6 = drive.trajectorySequenceBuilder(position)
//                .lineToLinearHeading(hubPosition)
//                .build();
//        drive.followTrajectorySequence(trajSeq6);
//
//        drive.dumpFreightTop();
//
//        position = drive.getLocalizer().getPoseEstimate();
//        Pose2d parkPosition = new Pose2d(new Vector2d(-61, -34), Math.toRadians(0));
//        TrajectorySequence trajSeq7 = drive.trajectorySequenceBuilder(position)
//                .lineToLinearHeading(parkPosition)
//                .build();
//        drive.followTrajectorySequence(trajSeq7);
//
//    }
//
//
//}
