package org.firstinspires.ftc.teamcode.Slidy_PPV2;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.Camera;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;


public class AprilTagDetectionSubsystem {

    public static final int LEFT = 483;
    public static final int MIDDLE = 484;
    public static final int RIGHT = 485;

    static final double FEET_PER_METER = 3.28084;

    Telemetry telemetry;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family


    AprilTagDetection tagOfInterest = null;
    boolean tagFound = false;
    protected Camera camera;

    public AprilTagDetectionSubsystem(Camera camera, Telemetry telemetry) {
        this.camera = camera;
        this.telemetry = telemetry;
    }

    public boolean aprilTagCheck() {
        ArrayList<AprilTagDetection> currentDetections = camera.getPipeline().getLatestDetections();
        if (currentDetections.size() != 0) {
            tagFound = false;
            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                    tagOfInterest = tag;
                    tagFound = true;
                    telemetry.addLine(String.format("\nDetected tag ID=%d", tagOfInterest.id));
                    telemetry.addLine(String.format("Translation X: %.2f feet", tagOfInterest.pose.x*FEET_PER_METER));
                    telemetry.addLine(String.format("Translation Y: %.2f feet", tagOfInterest.pose.y*FEET_PER_METER));
                    telemetry.addLine(String.format("Translation Z: %.2f feet", tagOfInterest.pose.z*FEET_PER_METER));
                    telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(tagOfInterest.pose.yaw)));
                    telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(tagOfInterest.pose.pitch)));
                    telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(tagOfInterest.pose.roll)));

                }
            }
        }
        return tagFound;
    }

    public AprilTagDetection getTagOfInterest() {
        return tagOfInterest;
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

}
