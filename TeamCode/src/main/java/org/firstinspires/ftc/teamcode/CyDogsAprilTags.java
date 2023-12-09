package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class CyDogsAprilTags {


    /*
     * This OpMode illustrates the basics of AprilTag recognition and pose estimation, using
     * the easy way.
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
     * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
     */

        private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

        /**
         * The variable to store our instance of the AprilTag processor.
         */
        private AprilTagProcessor aprilTag;

        /**
         * The variable to store our instance of the vision portal.
         */
        private VisionPortal visionPortal;

        private LinearOpMode myOpMode;
        public CyDogsAprilTags(LinearOpMode currentOpMode) {
            myOpMode = currentOpMode;
        }

        /**
         * Initialize the AprilTag processor.
         */
        public void initAprilTag(String webCamName) {

            // Create the AprilTag processor the easy way.
            aprilTag = AprilTagProcessor.easyCreateWithDefaults();

            // Create the vision portal the easy way.
            if (USE_WEBCAM) {
                visionPortal = VisionPortal.easyCreateWithDefaults(
                        myOpMode.hardwareMap.get(WebcamName.class, webCamName), aprilTag);
            } else {
                visionPortal = VisionPortal.easyCreateWithDefaults(
                        BuiltinCameraDirection.BACK, aprilTag);
            }


        }   // end method initAprilTag()
         public void initAprilTag(AprilTagProcessor aprilTagProcessor, VisionPortal visPortal)
         {
             aprilTag = aprilTagProcessor;
             visionPortal = visPortal;
         }

        public AprilTagDetection GetAprilTag(int tagID)
        {
            telemetryAprilTag();
            // Get the latest AprilTag detections from the pipeline.
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            // Iterate over the detections and return the one that matches the specified ID.
            for (AprilTagDetection aprilTagDetection : currentDetections) {
                if (aprilTagDetection.id == tagID) {
                    return aprilTagDetection;
                }
            }

            // If no matching detection is found, return null.
            return null;
        }

        /**
         * Add telemetry about AprilTag detections.
         */
        private void telemetryAprilTag() {

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            myOpMode.telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    myOpMode.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    myOpMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    myOpMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    myOpMode.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    myOpMode.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    myOpMode.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }


            }   // end for() loop

            // Add "key" information to telemetry
       //     myOpMode.telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        //    myOpMode.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
       //     myOpMode.telemetry.addLine("RBE = Range, Bearing & Elevation");
        //    myOpMode.telemetry.update();

        }   // end method telemetryAprilTag()


}
