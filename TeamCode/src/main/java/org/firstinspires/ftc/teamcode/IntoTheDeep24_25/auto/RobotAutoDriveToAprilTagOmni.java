package org.firstinspires.ftc.teamcode.IntoTheDeep24_25.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "AprilTag Detection Only", group = "Autonomous")
public class RobotAutoDriveToAprilTagOmni extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = -1; // Track any detected AprilTag

    @Override
    public void runOpMode() {

        // Initialize the AprilTag detection system
        initAprilTag();

        // Set exposure for webcam to reduce motion blur
        if (USE_WEBCAM) {
            setManualExposure(6, 250);
        }

        // Wait for the autonomous phase to begin
        telemetry.addData("Status", "Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            boolean targetFound = false;
            desiredTag = null;

            // Check for AprilTag detections
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    targetFound = true;
                    desiredTag = detection;
                    break;
                }
            }

            if (targetFound) {
                // Display tag information in the telemetry
                telemetry.addData("AprilTag Detected", "ID: %d", desiredTag.id);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("Status", "No AprilTag detected. Searching...");
            }

            telemetry.update();
            sleep(10); // Small delay to allow smooth updates
        }
    }

    // Initialize the AprilTag detection system
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(1);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    // Method to set manual exposure for reducing motion blur
    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
        }

        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}