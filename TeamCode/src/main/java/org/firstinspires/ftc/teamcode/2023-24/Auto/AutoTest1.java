//package org.firstinspires.ftc.teamcode.Auto;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.ServoImplEx;
//
//import org.firstinspires.ftc.teamcode.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.TeleOp.Visionportal3;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
//
///* velocity + acceleration limit command in trajectory movements:
//SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
// */
//
//@Autonomous(name = "AutoTest1 on January 11")
//public class AutoTest1 extends LinearOpMode {
//
//
//    // start pose with random estimated values; will change later
//    // we can also make 4 different pose2d values for each of the start positions so changing at start will be easier
//    Pose2d blueStart1 = new Pose2d(-37, 72, Math.toRadians(-90));
//    Pose2d blueStart2 = new Pose2d(37, 72, Math.toRadians(-90));
//    Pose2d redStart1 = new Pose2d(-37, -72, Math.toRadians(90));
//    Pose2d redStart2 = new Pose2d(37, -72, Math.toRadians(90));
//
//    Vector2d blue1LeftMark = new Vector2d(-48, 34);
//    Vector2d blue1CenterMark = new Vector2d(-35, 27);
//    Vector2d blue1RightMark = new Vector2d(-25, 34);
//
//    @Override
//    public void runOpMode() {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        drive.setPoseEstimate(blueStart1);
//
//        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
//        DcMotorEx liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
//        DcMotorEx liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
//        ServoImplEx armServo1 = hardwareMap.get(ServoImplEx.class, "armServo1");
//        ServoImplEx armServo2 = hardwareMap.get(ServoImplEx.class, "armServo2");
//
//        //            int liftStage0 = 0; // Setpoint for intaking pixles
//        //            int liftStage1 = -20; // Setpoint for clearance
//        //            int liftStage2 = -40; // Setpoint at first set line
//        //            int liftStage3 = -60; // Setpoint for highest we can go
//
//
//
//        // move robot to pixel stash (will change values later)
//        TrajectorySequence test = drive.trajectorySequenceBuilder(blueStart1)
//                // detect y-axis custom object from starting location
//                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
//                .lineToConstantHeading(blue1RightMark) // place white pixel on mark
//                .waitSeconds(0.25)
//                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
//
//                .lineToLinearHeading(new Pose2d(-35,57, Math.toRadians(0))) // start moving towards backboard for yellow pixel
//                .addDisplacementMarker(() -> {
//                    armServo1.setPosition(0.1);
//                    armServo2.setPosition(0.1);
//                })
//                .lineToConstantHeading(new Vector2d(10, 57))
//                .splineToConstantHeading(new Vector2d(54, 36), Math.toRadians(0))
//                .addSpatialMarker(new Vector2d(50, 36), () -> {
//                    liftMotor1.setPower(1.0); // lift before arm for enough clearance
//                    liftMotor2.setPower(1.0);
//
//                    armServo1.setPosition(1.0);
//                    armServo2.setPosition(1.0);
//                })
//                .waitSeconds(0.5) // drop yellow pixel
//                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
//                    armServo1.setPosition(0.1); // arm before lift for enough clearance on the way back down
//                    armServo2.setPosition(0.1);
//
//                    liftMotor1.setPower(0.0);
//                    liftMotor2.setPower(0.0);
//                })
//
//                .splineToConstantHeading(new Vector2d(10, 57), Math.toRadians(0)) // go back for two white pixels
//
//                .lineToConstantHeading(new Vector2d(-35, 57))
//                .splineToConstantHeading(new Vector2d(-68, 36), Math.toRadians(0))
//                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
//                    intakeMotor.setPower(1);
//                })
//                .waitSeconds(0.4) // intake two white pixels
//                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
//                    intakeMotor.setPower(0);
//                })
//                .splineToConstantHeading(new Vector2d(-35, 57), Math.toRadians(0))
//                .lineToConstantHeading(new Vector2d(10, 57))
//                .splineToConstantHeading(new Vector2d(54, 36), Math.toRadians(0))
//                .addSpatialMarker(new Vector2d(50, 36), () -> {
//                    liftMotor1.setPower(1.0);
//                    liftMotor2.setPower(1.0);
//
//                    armServo1.setPosition(1.0);
//                    armServo2.setPosition(1.0);
//
//                })
//                .waitSeconds(0.5) // drop two white pixels on backboard
//                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
//                    armServo1.setPosition(0.1);
//                    armServo2.setPosition(0.1);
//
//                    liftMotor1.setPower(0.0);
//                    liftMotor2.setPower(0.0);
//                })
//                .lineTo(new Vector2d(56, 15)) // park in a spot that the other team probably won't park
//                .build();
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        drive.followTrajectorySequence(test);
//    }
//}
