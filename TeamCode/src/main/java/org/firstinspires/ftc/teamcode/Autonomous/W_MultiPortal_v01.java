package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Disabled
@TeleOp(name = "W_MultiPortal_v01")
public class W_MultiPortal_v01 extends LinearOpMode {

    VisionPortal.Builder myVisionPortalBuilder;
    boolean USE_WEBCAM_1;
    int Portal_1_View_ID;
    boolean USE_WEBCAM_2;
    int Portal_2_View_ID;
    AprilTagProcessor myAprilTagProcessor_1;
    AprilTagProcessor myAprilTagProcessor_2;
    VisionPortal myVisionPortal_1;
    VisionPortal myVisionPortal_2;

    /**
     * Describe this function...
     */
    private void initMultiPortals() {
        List myPortalsList;

        myPortalsList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
        Portal_1_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 0, false)).intValue();
        Portal_2_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 1, false)).intValue();
        telemetry.addData("Portal 1 View ID (index 0 of myPortalsList)", Portal_1_View_ID);
        telemetry.addData("Portal 2 View ID (index 1 of myPortalsList)", Portal_2_View_ID);
        telemetry.addLine("");
        telemetry.addLine("Press Y to continue");
        telemetry.update();
        while (!gamepad1.y && opModeInInit()) {
            // Loop until gamepad Y button is pressed.
        }
    }

    /**
     * This Op Mode demonstrates MultiPortalView.
     *
     * The Dpad buttons can turn each camera stream on and off.
     * USB bandwidth is more restricted on an external USB hub, compared to the Control Hub.
     */
    @Override
    public void runOpMode() {
        // This OpMode shows AprilTag recognition and pose estimation.
        USE_WEBCAM_1 = true;
        USE_WEBCAM_2 = true;
        initMultiPortals();
        // Initialize AprilTag before waitForStart.
        initAprilTag();
        // Wait for the Start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                AprilTag_telemetry_for_Portal_1();
                AprilTag_telemetry_for_Portal_2();
                AprilTag_telemetry_legend();
                Toggle_camera_streams();
                // Push telemetry to the Driver Station.
                telemetry.update();
                // Share the CPU.
                sleep(20);
            }
        }
    }

    /**
     * Initialize AprilTag Detection.
     */
    private void initAprilTag() {
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;

        // First, create an AprilTagProcessor.Builder.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        // Create each AprilTagProcessor by calling build.
        myAprilTagProcessor_1 = myAprilTagProcessorBuilder.build();
        myAprilTagProcessor_2 = myAprilTagProcessorBuilder.build();
        Make_first_VisionPortal();
        Make_second_VisionPortal();
    }

    /**
     * Describe this function...
     */
    private void Make_first_VisionPortal() {
        // Create a VisionPortal.Builder and set attributes related to the first camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM_1) {
            // Use a webcam.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Manage USB bandwidth of two camera streams, by adjusting resolution from default 640x480.
        // Set the camera resolution.
        myVisionPortalBuilder.setCameraResolution(new Size(640, 480));
        // Manage USB bandwidth of two camera streams, by selecting Streaming Format.
        // Set the stream format.
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        // Add myAprilTagProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor_1);
        // Add the Portal View ID to the VisionPortal.Builder
        // Set the camera monitor view id.
        myVisionPortalBuilder.setLiveViewContainerId(Portal_1_View_ID);
        // Create a VisionPortal by calling build.
        myVisionPortal_1 = myVisionPortalBuilder.build();
    }

    /**
     * Describe this function...
     */
    private void Make_second_VisionPortal() {
        if (USE_WEBCAM_2) {
            // Use a webcam.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Manage USB bandwidth of two camera streams, by adjusting resolution from default 640x480.
        // Set the camera resolution.
        myVisionPortalBuilder.setCameraResolution(new Size(640, 480));
        // Manage USB bandwidth of two camera streams, by selecting Streaming Format.
        // Set the stream format.
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        // Add myAprilTagProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor_2);
        // Add the Portal View ID to the VisionPortal.Builder
        // Set the camera monitor view id.
        myVisionPortalBuilder.setLiveViewContainerId(Portal_2_View_ID);
        // Create a VisionPortal by calling build.
        myVisionPortal_2 = myVisionPortalBuilder.build();
    }

    /**
     * Describe this function...
     */
    private void Toggle_camera_streams() {
        // Manage USB bandwidth of two camera streams, by turning on or off.
        if (gamepad1.dpad_down) {
            // Temporarily stop the streaming session. This can save CPU
            // resources, with the ability to resume quickly when needed.
            myVisionPortal_1.stopStreaming();
        } else if (gamepad1.dpad_up) {
            // Resume the streaming session if previously stopped.
            myVisionPortal_1.resumeStreaming();
        }
        if (gamepad1.dpad_left) {
            // Temporarily stop the streaming session. This can save CPU
            // resources, with the ability to resume quickly when needed.
            myVisionPortal_2.stopStreaming();
        } else if (gamepad1.dpad_right) {
            // Resume the streaming session if previously stopped.
            myVisionPortal_2.resumeStreaming();
        }
    }

    /**
     * Display info (using telemetry) for a recognized AprilTag.
     */
    private void AprilTag_telemetry_for_Portal_1() {
        List<AprilTagDetection> myAprilTagDetections_1;
        AprilTagDetection thisDetection_1;

        // Get a list of AprilTag detections.
        myAprilTagDetections_1 = myAprilTagProcessor_1.getDetections();
        telemetry.addData("Portal 1 - # AprilTags Detected", JavaUtil.listLength(myAprilTagDetections_1));
        // Iterate through list and call a function to display info for each recognized AprilTag.
        for (AprilTagDetection thisDetection_1_item : myAprilTagDetections_1) {
            thisDetection_1 = thisDetection_1_item;
            // Display info about the detection.
            telemetry.addLine("");
            if (thisDetection_1.metadata != null) {
                telemetry.addLine("==== (ID " + thisDetection_1.id + ") " + thisDetection_1.metadata.name);
                telemetry.addLine("XYZ " + JavaUtil.formatNumber(thisDetection_1.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.z, 6, 1) + "  (inch)");
                telemetry.addLine("PRY " + JavaUtil.formatNumber(thisDetection_1.ftcPose.yaw, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.roll, 6, 1) + "  (deg)");
                telemetry.addLine("RBE " + JavaUtil.formatNumber(thisDetection_1.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_1.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
            } else {
                telemetry.addLine("==== (ID " + thisDetection_1.id + ") Unknown");
                telemetry.addLine("Center " + JavaUtil.formatNumber(thisDetection_1.center.x, 6, 0) + "" + JavaUtil.formatNumber(thisDetection_1.center.y, 6, 0) + " (pixels)");
            }
        }
    }

    /**
     * Display info (using telemetry) for a recognized AprilTag.
     */
    private void AprilTag_telemetry_for_Portal_2() {
        List<AprilTagDetection> myAprilTagDetections_2;
        AprilTagDetection thisDetection_2;

        // Get a list of AprilTag detections.
        myAprilTagDetections_2 = myAprilTagProcessor_2.getDetections();
        telemetry.addLine("");
        telemetry.addData("Portal 2 - # AprilTags Detected", JavaUtil.listLength(myAprilTagDetections_2));
        // Iterate through list and call a function to display info for each recognized AprilTag.
        for (AprilTagDetection thisDetection_2_item : myAprilTagDetections_2) {
            thisDetection_2 = thisDetection_2_item;
            // Display info about the detection.
            telemetry.addLine("");
            if (thisDetection_2.metadata != null) {
                telemetry.addLine("==== (ID " + thisDetection_2.id + ") " + thisDetection_2.metadata.name);
                telemetry.addLine("XYZ " + JavaUtil.formatNumber(thisDetection_2.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.z, 6, 1) + "  (inch)");
                telemetry.addLine("PRY " + JavaUtil.formatNumber(thisDetection_2.ftcPose.yaw, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.roll, 6, 1) + "  (deg)");
                telemetry.addLine("RBE " + JavaUtil.formatNumber(thisDetection_2.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(thisDetection_2.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
            } else {
                telemetry.addLine("==== (ID " + thisDetection_2.id + ") Unknown");
                telemetry.addLine("Center " + JavaUtil.formatNumber(thisDetection_2.center.x, 6, 0) + "" + JavaUtil.formatNumber(thisDetection_2.center.y, 6, 0) + " (pixels)");
            }
        }
    }

    /**
     * Describe this function...
     */
    private void AprilTag_telemetry_legend() {
        telemetry.addLine("");
        telemetry.addLine("key:");
        telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
}
