package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCompatibilityManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.usb.UsbConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * including Java Builder structures for specifying Vision parameters.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will have their position and orientation information displayed.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * To experiment with using AprilTags to navigate, try out these two driving samples:
 * RobotAutoDriveToAprilTagOmni and RobotAutoDriveToAprilTagTank
 *
 * There are many "default" VisionPortal and AprilTag configuration parameters that may be overridden if desired.
 * These default parameters are shown as comments in the code below.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Convert", group = "Concept")

public class GreenZone extends LinearOpMode {

    private static final boolean USE_WEBCAM = true; // true for webcam, false for phone camera

    /*
     * Shot limits - these values are not final
     * TODO - tune once the turret is done
     */

    static final double MAX_YAW_DEG = 20; // side-of-hive limit
    static final double MAX_PITCH_DEG = 25; // under-hive limit
    static final double MAX_ROLL_DEG = 90; // this means its flipped 

    static final double IDEAL_RANGE = 50; // inches
    static final double IDEAL_BEARING = 5; // degrees
    static final double IDEAL_ELEVATION = 15; // degrees
    static final double IDEAL_YAW = 0; // degrees

    /**
     * 
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    /**
     * The variable to store our instance of the current april tag(s)
     */
    private AprilTagDetection targetDetection;

    /**
     * The variable to store how much of the current tag cluster was found
     */
    private double targetClusterPercent = 0;

    static final int VENDOR_ID_SUNPLUS_INNOVATION_TECHNOLOGY = 0x1BCF;
    static final int PRODUCT_ID_ARDUCAM_OV5648 = 0x284C;

    @Override
    public void runOpMode() {

        CameraCompatibilityManager.getInstance()
                .addQuirk(
                        VENDOR_ID_SUNPLUS_INNOVATION_TECHNOLOGY,
                        PRODUCT_ID_ARDUCAM_OV5648,
                        CameraCompatibilityManager.Quirk.AVOID_LIB_USB_RESET_DEVICE);

        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            telemetryAprilTag();

            // Push telemetry to the Driver Station.
            telemetry.update();

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            // Share the CPU.
            sleep(20);
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    } // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as
                // needed.
                // .setDrawAxes(true) // Changed in V12.0
                // .setDrawTagOutline(true)
                // .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                // .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                // .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                // .setDrawCubeProjection(false)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(545.584, 545.584, 350.588, 223.15)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 .. Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 .. Detect 2" Tag from 6 feet away at 22 Frames per second
        // Decimation = 3 .. Detect 2" Tag from 4 feet away at 30 Frames Per Second
        // (default)
        // Decimation = 3 .. Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        // aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            builder.setCameraResolution(new Size(640, 480));
            builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        // builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView). Set "false" to omit camera monitoring.
        // builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        // builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        // builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        // visionPortal.setProcessorEnabled(aprilTag, true);

    } // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {
        targetDetection = null;
        targetClusterPercent = 0;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                if (singleDet.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", singleDet.id, singleDet.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x,
                            detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch,
                            detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range,
                            detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", singleDet.id));
                    telemetry.addLine(
                            String.format("Center %6.0f %6.0f   (pixels)", singleDet.center.x, singleDet.center.y));
                }
            } else {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                targetDetection = detection;
                targetClusterPercent = clusterDet.percentClusterFound;
                /* 
                if (Math.abs(detection.ftcPose.roll) < 90) {
                    
                }
                */
                telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x,
                        detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch,
                        detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range,
                        detection.ftcPose.bearing, detection.ftcPose.elevation));
            }
        } // end for() loop

        // Add "key" information to telemetry
        //telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        //telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        //telemetry.addLine("RBE = Range, Bearing & Elevation");

        telemetry.addLine("");

        if (isGreen()) {
            telemetry.addLine("SHOOT");
        } else {
            telemetry.addLine("WAIT");
        }

    } // end method telemetryAprilTag()

    private boolean isGreen() {

        if (targetDetection == null || targetDetection.ftcPose == null) {
            return false;
        }

        double shotScore = calculateShotScore();
        telemetry.addLine(shotScore * 100 + "%");
        return shotScore >= 0.75;

    } // end method isGreen()

    private double calculateShotScore() {
        /*
         * These values are not final
         * TODO - test for ideal shooting position & other numbers
         */

        AprilTagPoseFtc pose = targetDetection.ftcPose;

        if (Math.abs(pose.yaw) > MAX_YAW_DEG) {
            telemetry.addLine("side of hive");
            return 0; // side of the hive. Yaw is based on the x tilt of tag
        }
        if (Math.abs(pose.pitch) > MAX_PITCH_DEG) {
            telemetry.addLine("under hive");
            return 0; // under the hive. up and down tilt
        }
        if (Math.abs(pose.roll) > MAX_ROLL_DEG) {
            telemetry.addLine("flipped");
            return 0; // flipped. If it is upside down then it is rotated over and is flipped
        }

        // the 6 4s and 10 are the weights
        double rangeError = (pose.range - IDEAL_RANGE) / 6.0;
        double bearingError = (pose.bearing - IDEAL_BEARING) / 4.0;
        double elevationError = (pose.elevation - IDEAL_ELEVATION) / 4.0;
        double yawError = (pose.yaw - IDEAL_YAW) / 10.0;

        double error = Math.sqrt(
                rangeError * rangeError +
                bearingError * bearingError +
                elevationError * elevationError +
                yawError * yawError);

        double k = 0.03; // smaller # = longer before percent drop

        double score = Math.exp(-k * error * error);

        return score;
    } // end method calculateShotScore()

} // end class
