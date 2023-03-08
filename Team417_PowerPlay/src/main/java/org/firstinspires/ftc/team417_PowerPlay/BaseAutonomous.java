package org.firstinspires.ftc.team417_PowerPlay;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

abstract public class BaseAutonomous extends BaseOpMode {

    OpenCvCamera camera; // calls camera
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    static final int CAMERA_WIDTH_PIXELS = 800;
    static final int CAMERA_HEIGHT_PIXELS = 448;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagSize = 0.166;

    // Tag ID 1, 2, 3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    AprilTagDetection tagOfInterest = null;

    static final double HOLD_ARM_AT_MID_OR_LOW_POS_POWER = 0.005;
    static final double HOLD_ARM_AT_GROUND_POS_POWER = 0.01;
    static final double ARM_RAISE_POWER = 1.0 / 400.0;

    public void initializeAuto() {
        initializeHardware();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH_PIXELS, CAMERA_HEIGHT_PIXELS, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) { }
        });

        telemetry.setMsTransmissionInterval(50);
    }

    /**
     * Detects current tag and gives telemetry of tag ID
     */
    public void detectAprilTag() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.size() != 0) {
            boolean tagFound = false;

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if (tagFound) {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }
        }
        telemetry.update();
        sleep(20);
    }
    /**
     * Updates telemetry: updates the id if no april tag is sighted
     */
    public void updateTelemetryAfterStart() {
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
    }

    public void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public void raiseAndHoldArmGroundJunctionPosition() {
        motorArm.setPower(0);
        while (Math.abs(motorArm.getCurrentPosition() - GROUND_JUNCT_ARM_POSITION) > ARM_ENCODER_TOLERANCE && opModeIsActive()) {
            motorArm.setPower((GROUND_JUNCT_ARM_POSITION - motorArm.getCurrentPosition()) * ARM_RAISE_POWER);
        }
        motorArm.setPower(HOLD_ARM_AT_GROUND_POS_POWER);
    }

    public void raiseAndHoldArmMiddleJunctionPosition() {
        motorArm.setPower(0);
        while ((Math.abs(motorArm.getCurrentPosition() - (MID_JUNCT_ARM_POSITION)) > ARM_ENCODER_TOLERANCE) && opModeIsActive()) {
            motorArm.setPower((MID_JUNCT_ARM_POSITION - motorArm.getCurrentPosition()) * ARM_RAISE_POWER);
        }
        motorArm.setPower(HOLD_ARM_AT_MID_OR_LOW_POS_POWER);
    }

    public void raiseAndHoldArmLowJunctionPosition() {
        motorArm.setPower(0);
        while (Math.abs(motorArm.getCurrentPosition() - LOW_JUNCT_ARM_POSITION) > ARM_ENCODER_TOLERANCE) {
            motorArm.setPower((LOW_JUNCT_ARM_POSITION - motorArm.getCurrentPosition()) * ARM_RAISE_POWER);
        }
        motorArm.setPower(HOLD_ARM_AT_MID_OR_LOW_POS_POWER);
    }
}
