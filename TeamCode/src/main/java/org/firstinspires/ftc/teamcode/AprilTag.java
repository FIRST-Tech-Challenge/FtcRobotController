package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTag {

    private Robot robot;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private boolean verbose = false;

    public AprilTag(Robot robot) {
        this.robot = robot;
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(robot.webcam, aprilTag);
    }

    public int findTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (verbose) {
                    robot.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    robot.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    robot.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    robot.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                }
                return(detection.id);
            } else {
                if (verbose) {
                    robot.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    robot.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
                return(detection.id);
            }
        }   // end for() loop
        return (-1);
    }
}
