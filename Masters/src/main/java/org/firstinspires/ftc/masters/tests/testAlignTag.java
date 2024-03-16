//package org.firstinspires.ftc.masters.tests;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.masters.PropFindProcessor;
//import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.List;
//
//@Autonomous(name = "Test Align Tag")
//public class testAlignTag extends LinearOpMode {
//
//    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
//
//    private AprilTagProcessor aprilTag;
//
//    private PropFindProcessor propFindProcessor;
//
//    TelemetryPacket packet = new TelemetryPacket();
//
//    private VisionPortal myVisionPortal;
//
//    SampleMecanumDrive drive;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        drive = new SampleMecanumDrive(hardwareMap);
//
//        // -----------------------------------------------------------------------------------------
//            // AprilTag Configuration
//            // -----------------------------------------------------------------------------------------
//
//        aprilTag = new AprilTagProcessor.Builder()
//                .setDrawAxes(true)
//                .setDrawCubeProjection(true)
//                .setDrawTagOutline(true)
//                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                //.setTagLibrary((SkystoneDatabase.SkystoneDatabase()))
//                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//                .build();
//
//        // -----------------------------------------------------------------------------------------
//        // TFOD Configuration
//        // -----------------------------------------------------------------------------------------
//
//        propFindProcessor = new PropFindProcessor(telemetry,packet);
//
//        // -----------------------------------------------------------------------------------------
//        // Camera Configuration
//        // -----------------------------------------------------------------------------------------
//
//        if (USE_WEBCAM) {
//            myVisionPortal = new VisionPortal.Builder()
//                    .setCamera(hardwareMap.get(WebcamName.class, "backWebcam"))
//                    .addProcessors(propFindProcessor, aprilTag)
//                    .build();
//        } else {
//            myVisionPortal = new VisionPortal.Builder()
//                    .setCamera(BuiltinCameraDirection.BACK)
//                    .addProcessors(propFindProcessor, aprilTag)
//                    .build();
//        }
//
//        myVisionPortal.setProcessorEnabled(propFindProcessor, false);
//
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//        }   // end for() loop
//
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            currentDetections = aprilTag.getDetections();
//            for (AprilTagDetection detection : currentDetections) {
//                if (detection.metadata != null) {
//                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//                } else {
//                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//                }
//            }   // end for() loop
//            telemetry.addData("April Tag Pos Es", drive.aprilTagCoarsePosEstimate(currentDetections));
//            if (gamepad1.a) {
//                drive.turnToTag(currentDetections, 1);
//                sleep(200);
//            }
//
//            if (gamepad1.b) {
//                drive.strafeToTag(currentDetections, 1);
//                sleep(200);
//            }
//
//            telemetry.update();
//        }
//    }
//}
