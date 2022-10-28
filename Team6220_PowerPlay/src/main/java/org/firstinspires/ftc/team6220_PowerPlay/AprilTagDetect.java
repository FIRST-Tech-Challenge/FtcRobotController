package org.firstinspires.ftc.team6220_PowerPlay;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Locale;

public abstract class AprilTagDetect extends BaseAutonomous {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // units are pixels
    // calibration is for Logitech C920 webcam at 1920 x 1080
    final double fx = 1387.853; // focal length x
    final double fy = 1387.853; // focal length y
    final double cx = 960; // camera principal point x
    final double cy = 540; // camera principal point y

    // units are meters
    final double tagSize = 0.03429;

    final int ID_TAG_OF_INTEREST_0 = 0; // tag 0 from the 36h11 family
    final int ID_TAG_OF_INTEREST_1 = 1; // tag 1 from the 36h11 family
    final int ID_TAG_OF_INTEREST_2 = 2; // tag 2 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    public int detectAprilTag() {

        // initializes camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1920,1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        //initialize();

        // replaces waitForStart()
        // detects AprilTags during initialization
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            // tag has been detected at one point
            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                // finds out which tag is detected
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == ID_TAG_OF_INTEREST_0 || tag.id == ID_TAG_OF_INTEREST_1 || tag.id == ID_TAG_OF_INTEREST_2) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                // tag has been detected and is still being detected
                if (tagFound) {
                    telemetry.addLine("Tag found!\n\nLocation data:\n");
                    tagToTelemetry(tagOfInterest);

                    // tag was detected at some point but isn't currently being detected
                } else {
                    telemetry.addLine("Tag not found :(\n\nBut the tag has been sen before, last seen at:\n");
                    tagToTelemetry(tagOfInterest);
                }

            // tag has never been detected
            } else {
                telemetry.addLine("Don't see tag of interest :(\n");
                telemetry.addLine("The tag has never been seen");
            }

            telemetry.update();
        }

        // tag was detected during initialization
        if (tagOfInterest != null) {
            telemetry.addLine("Tag seen!\n\nLocation data:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();

        // tag was not detected during initialization
        } else {
            telemetry.addLine("No tag seen during initialization :(");
            telemetry.update();
        }

        // return default
        if (tagOfInterest == null) {
            return 1;
        } else {
            return tagOfInterest.id;
        }
    }

    // displays tag details
    public void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format(Locale.US, "Detected tag ID: %d", detection.id));
        telemetry.addLine(String.format(Locale.US, "X distance: %d inches", (int) (detection.pose.x * Constants.INCHES_PER_METER)));
        telemetry.addLine(String.format(Locale.US, "Y Distance: %d inches", (int) (detection.pose.y * Constants.INCHES_PER_METER)));
        telemetry.addLine(String.format(Locale.US, "Z Distance: %d inches", (int) (detection.pose.z * Constants.INCHES_PER_METER)));
        telemetry.addLine(String.format(Locale.US, "Yaw Rotation: %d degrees", (int) (Math.toDegrees(detection.pose.yaw))));
        telemetry.addLine(String.format(Locale.US, "Pitch Rotation: %d degrees", (int) (Math.toDegrees(detection.pose.pitch))));
        telemetry.addLine(String.format(Locale.US, "Roll Rotation: %d degrees", (int) (Math.toDegrees(detection.pose.roll))));
    }
}
