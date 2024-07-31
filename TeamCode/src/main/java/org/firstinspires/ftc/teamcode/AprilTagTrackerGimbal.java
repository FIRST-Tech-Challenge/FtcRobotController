package org.firstinspires.ftc.teamcode;  //place where the code is located

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



import java.util.List;

//import global.first.FeedingTheFutureGameDatabase;


public class AprilTagTrackerGimbal{

    //crating objects so that they could be mapped later on when initAprilTag is called
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private static AprilTagLibrary getFeedingTheFutureTagLibrary() {
        return (new AprilTagLibrary.Builder())
                .addTag(100, "Blue Nexus Goal - Field Center - Facing Platform", 160.0, DistanceUnit.MM)
                .addTag(101, "Red Nexus Goal - Field Center - Facing Platform", 160.0, DistanceUnit.MM)
                .addTag(102, "Red Nexus Goal - Field Center - Facing Food Warehouse", 160.0, DistanceUnit.MM)
                .addTag(103, "Blue Nexus Goal - Field Center - Facing Food Warehouse", 160.0, DistanceUnit.MM)
                .addTag(104, "Blue Nexus Goal - Field Edge - Alliance Station", 160.0, DistanceUnit.MM)
                .addTag(105, "Blue Nexus Goal - Field Edge - Center Field", 160.0, DistanceUnit.MM)
                .addTag(106, "Red Nexus Goal - Field Edge - Center Field", 160.0, DistanceUnit.MM)
                .addTag(107, "Red Nexus Goal - Field Edge - Alliance Station", 160.0, DistanceUnit.MM)
                .build();
    }

    private static final AprilTagLibrary feedingTheFutureTagLibrary = getFeedingTheFutureTagLibrary();


    private static final boolean USE_WEBCAM = true;

        public void initAprilTag(HardwareMap hardwareMapPorted, Telemetry telemetryPorted) {

        hardwareMap = hardwareMapPorted;
        telemetry = telemetryPorted;
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setTagLibrary(feedingTheFutureTagLibrary)
            //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
            .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)

            // The following default settings are available to un-comment and edit as needed.
            //.setDrawAxes(false)
            //.setDrawCubeProjection(false)
            //.setDrawTagOutline(true)
            //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

            // == CAMERA CALIBRATION ==
            // If you do not manually specify calibration parameters, the SDK will attempt
            // to load a predefined calibration for your camera.
            //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
            // ... these parameters are fx, fy, cx, cy.

            .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    public void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (cm)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (degree)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (cm, degree, degree)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }   // end method telemetryAprilTag()
}