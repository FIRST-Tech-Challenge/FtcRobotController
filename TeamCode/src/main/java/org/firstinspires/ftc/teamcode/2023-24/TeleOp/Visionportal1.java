//package org.firstinspires.ftc.teamcode.TeleOp;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import android.util.Size;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
//
//import java.util.*;
//
//// LiveView refers only to the Robot Controller preview (example shown above). It’s completely separate from Driver Station Camera Stream, which still operates normally even if LiveView is stopped (manually or automatically).
//@TeleOp(name = "Visionportal1")
//public class Visionportal1 extends LinearOpMode {
//
//    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
//    private TfodProcessor myTfodProcessor;
//    private AprilTagProcessor myAprilTagProcessor;
//    private VisionPortal myVisionPortal;
//
//    private void initProcessors() {
//        // Tfod = tensorflow object detection
//        myTfodProcessor = new TfodProcessor.Builder()
//                .setMaxNumRecognitions(10) // max # recognitions
//                .setUseObjectTracker(true) // use object tracker
//                .setTrackerMaxOverlap((float) 0.2) // max % of box overlapped by another box for recognition
//                .setTrackerMinSize(16) // minimum size of a tracked/recognized object (units?)
//                .build();
//
//        myAprilTagProcessor = new AprilTagProcessor.Builder()
//                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()) // currentgametaglibrary = centerstage + sample apriltags
//                .setDrawTagID(true)
//                .setDrawTagOutline(true)
//                .setDrawAxes(true) // default = false
//                .setDrawCubeProjection(true) // default = false
//                .build();
//
//
//        if (USE_WEBCAM) {
//            myVisionPortal = new VisionPortal.Builder()
//                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // change name depending on mech team wants to name it
//                    .addProcessor(myAprilTagProcessor) // add apriltag processor
//                    .addProcessor(myTfodProcessor) // add tfod processor
//                    .setCameraResolution(new Size(640, 480)) // import android.util.Size;
//                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // changed to MJPEG instead of YUY2 since it uses less bandwidth
//                    // .enableCameraMonitoring(true) // enableCameraMonitoring not identified? I also didn't find any related methods in VisionPortal.java
//                    .enableLiveView(true) // manually added this method because it was the closest thing to enableCameraMonitoring
//                    .setAutoStopLiveView(true)
//                    .build();
//        } else {
//            myVisionPortal = new VisionPortal.Builder()
//                    .setCamera(BuiltinCameraDirection.BACK)
//                    .addProcessor(myAprilTagProcessor) // add apriltag processor
//                    .addProcessor(myTfodProcessor) // add tfod processor
//                    .setCameraResolution(new Size(640, 480)) // import android.util.Size;
//                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // changed to MJPEG instead of YUY2 since it uses less bandwidth
//                    // .enableCameraMonitoring(true) // enableCameraMonitoring not identified? I also didn't find any related methods in VisionPortal.java
//                    .enableLiveView(true) // manually added this method because it was the closest thing to enableCameraMonitoring
//                    .setAutoStopLiveView(true)
//                    .build();
//        }
//    }
//
//    @Override
//    public void runOpMode() {
//        initProcessors();
//        waitForStart();
//
//        if (!isStopRequested()) {
//            while(opModeIsActive()) {
//
//                telemetryVision();
//                telemetry.update();
//
//                if (gamepad1.dpad_down) {
//                    myVisionPortal.stopStreaming();
//                } else if (gamepad1.dpad_up) {
//                    myVisionPortal.resumeStreaming();
//                }
//
//                sleep(100); // no rapid toggling since turning on and off takes time
//
//            }
//        }
//        myVisionPortal.setProcessorEnabled(myTfodProcessor,false);
//        myVisionPortal.setProcessorEnabled(myAprilTagProcessor,false);
//        myVisionPortal.close();
//
//    }
//
//    private void telemetryVision() {
//
//        List<AprilTagDetection> currentDetections = myAprilTagProcessor.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());
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
//        }
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//        telemetry.addLine("RBE = Range, Bearing & Elevation");
//
//    }
//
//}
