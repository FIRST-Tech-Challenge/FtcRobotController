//package org.firstinspires.ftc.teamcode.Auto;
//
//import android.util.Size;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.ServoImplEx;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
//import org.firstinspires.ftc.teamcode.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.vision.VisionPortal;
//
//import java.util.concurrent.TimeUnit;
//
//@Autonomous(name="Auto Red2")
//public class AutoRed2 extends LinearOpMode {
//
//    Apriltag aprilTagProcessor = new Apriltag("redTeam");
//    final double DISTANCE_FROM_TAG = 6.0;
//    double increase_in_y_units = 0.0;
//
//    Tfod tfodProcessor = new Tfod();
//    private static final boolean USE_WEBCAM = true;
//    private VisionPortal visionPortal;
//
//
//
//    Pose2d redStart2 = new Pose2d(12, -62, Math.toRadians(90));
//
//    Vector2d red2RightMark = new Vector2d(1, -34);
//    Vector2d red2CenterMark = new Vector2d(12, -27);
//    Vector2d red2LeftMark = new Vector2d(22, -34);
//
//    Vector2d desiredMark;
//
//
//    private void initVision() {
//
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // change name depending on mech team wants to name it
//                .addProcessor(aprilTagProcessor.getAprilTagProcessor())
//                .addProcessor(tfodProcessor.getTfodProcessor())
//                .setCameraResolution(new Size(640, 480)) // import android.util.Size;
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // changed to MJPEG instead of YUY2 since it uses less bandwidth
//                // .enableCameraMonitoring(true) // enableCameraMonitoring not identified? I also didn't find any related methods in VisionPortal.java
//                .enableLiveView(true) // manually added this method because it was the closest thing to enableCameraMonitoring
//                .setAutoStopLiveView(true)
//                .build();
//
//
//        setManualExposure(6, 250);
//    }
//
//    private void setManualExposure(int exposureMS, int gain) {
//        // Wait for the camera to be open, then use the controls
//
//        if (visionPortal == null) {
//            return;
//        }
//
//        // Make sure camera is streaming before we try to set the exposure controls
//        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addData("Camera", "Waiting");
//            telemetry.update();
//            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                sleep(20);
//            }
//            telemetry.addData("Camera", "Ready");
//            telemetry.update();
//        }
//
//        // Set camera controls unless we are stopping.
//        if (!isStopRequested()) {
//            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//                exposureControl.setMode(ExposureControl.Mode.Manual);
//                sleep(50);
//            }
//            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
//            sleep(20);
//            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//            gainControl.setGain(gain);
//            sleep(20);
//        }
//    }
//
//    @Override
//    public void runOpMode() {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        drive.setPoseEstimate(redStart2);
//
//        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
//        DcMotorEx liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
//        DcMotorEx liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
//        ServoImplEx armServo1 = hardwareMap.get(ServoImplEx.class, "armServo1");
//        ServoImplEx armServo2 = hardwareMap.get(ServoImplEx.class, "armServo2");
//
//        if (tfodProcessor.getDirection() == "left") {
//            desiredMark = red2LeftMark;
//            increase_in_y_units = 4.5;
//        } else if (tfodProcessor.getDirection() == "center") {
//            desiredMark = red2CenterMark;
//            increase_in_y_units = 0.0;
//        } else if (tfodProcessor.getDirection() == "right") {
//            desiredMark = red2RightMark;
//            increase_in_y_units = -4.5;
//        }
//
//        // set proper apriltag direction:
//        aprilTagProcessor.setTagID(tfodProcessor.getDirection());
//        Vector2d aprilTagLocation = new Vector2d(54 + aprilTagProcessor.getTagRange() - DISTANCE_FROM_TAG, -36 + increase_in_y_units);
//        // ^ this is one of the most mathematically questionable code I have ever written, but alas (the robot will still function fine in theory, but the math is not the best)
//
//
//        TrajectorySequence test = drive.trajectorySequenceBuilder(redStart2)
//                // detect y-axis custom object from starting location
//                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH)) // so it doesn't just go too fast
//                .lineToConstantHeading(desiredMark) // place white pixel on mark
//                .waitSeconds(0.25)
//                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
//
//                .lineToLinearHeading(new Pose2d(35,-57, Math.toRadians(0))) // start moving towards backboard for yellow pixel
//                .addDisplacementMarker(() -> {
//                    armServo1.setPosition(0.1);
//                    armServo2.setPosition(0.1);
//                })
//
//
//
//                .splineToConstantHeading(aprilTagLocation, Math.toRadians(0)) // arrive at backboard for yellow pixel
//                .addSpatialMarker(new Vector2d(50, -36), () -> {
//                    liftMotor1.setPower(1.0); // lift before arm for enough clearance
//                    liftMotor2.setPower(1.0);
//
//                    armServo1.setPosition(1.0);
//                    armServo2.setPosition(1.0);
//                })
//                .waitSeconds(0.35) // drop yellow pixel
//                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
//                    armServo1.setPosition(0.1); // arm before lift for enough clearance on the way back down
//                    armServo2.setPosition(0.1);
//
//                    liftMotor1.setPower(0.0);
//                    liftMotor2.setPower(0.0);
//                })
//
//                .splineToConstantHeading(new Vector2d(10, -57), Math.toRadians(0)) // go back for two white pixels
//
//                .lineToConstantHeading(new Vector2d(-35, -57))
//                .splineToLinearHeading(new Pose2d(-68, -36, Math.toRadians(0)), Math.toRadians(179))
//                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
//                    intakeMotor.setPower(1);
//                })
//                .waitSeconds(0.75) // intake two white pixels
//                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
//                    intakeMotor.setPower(0);
//                })
//                .splineToLinearHeading(new Pose2d(-35, -57, Math.toRadians(179)), Math.toRadians(0))
//                .lineToConstantHeading(new Vector2d(10, -57))
//                .splineToConstantHeading(aprilTagLocation, Math.toRadians(0))
//                .addSpatialMarker(new Vector2d(50, -36), () -> {
//                    liftMotor1.setPower(1.0);
//                    liftMotor2.setPower(1.0);
//
//                    armServo1.setPosition(1.0);
//                    armServo2.setPosition(1.0);
//
//                })
//                .waitSeconds(0.35) // drop two white pixels on backboard
//                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
//                    armServo1.setPosition(0.1);
//                    armServo2.setPosition(0.1);
//
//                    liftMotor1.setPower(0.0);
//                    liftMotor2.setPower(0.0);
//                })
//                .lineTo(new Vector2d(56, -15)) // park in a spot that the other team probably won't park
//                .build();
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        drive.followTrajectorySequence(test);
//    }
//
//
//}
//
