package com.wilyworks.simulator.framework;

import com.acmerobotics.roadrunner.Pose2d;
import com.wilyworks.common.WilyWorks;
import com.wilyworks.simulator.WilyCore;
import com.wilyworks.simulator.helpers.Globals;
import com.wilyworks.simulator.helpers.Point;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

/**
 * Wily Works implementation of the AprilTagProcessor interface.
 */
public class WilyAprilTagProcessor extends AprilTagProcessor {
    final double MAX_FPS = 20.0; // Maximum FPS
    WilyWorks.Config.Camera cameraDescriptor; // Describes the camera placement on the robot, if any
    double lastDetectionTime = WilyCore.time(); // Time of the last fresh detection
    ArrayList<AprilTagDetection> lastDetections = new ArrayList<>(); // A copy of the last detections provided
    AprilTagMetadata[] tags = AprilTagGameDatabase.getCenterStageTagLibrary().getAllTags(); // Tag database

    public WilyAprilTagProcessor(double fx, double fy, double cx, double cy, DistanceUnit outputUnitsLength, AngleUnit outputUnitsAngle, AprilTagLibrary tagLibrary, boolean drawAxes, boolean drawCube, boolean drawOutline, boolean drawTagID, TagFamily tagFamily, int threads) {
    }

    // WilyVisionPortal calls 'initialize' to associate the camera:
    public void initialize(WilyWorks.Config.Camera wilyCamera) {
        this.cameraDescriptor = wilyCamera;
    }

    // Create a detection record for the specified tag. References for understanding these values:
    //   https://ftc-docs.firstinspires.org/apriltag-detection-values
    //   https://ftc-docs-cdn.ftclive.org/booklets/en/april_tags.pdf
    //   https://javadoc.io/doc/org.firstinspires.ftc/Vision/latest/org/firstinspires/ftc/vision/apriltag/AprilTagDetection.html
    // Most useful is the inline code documentation for the AprilTagDetection class.
    @SuppressWarnings("SuspiciousNameCombination")
    AprilTagDetection createDetection(AprilTagMetadata tag, double cameraFieldAngle, Point fieldVectorToTag) {
        Orientation tagOrientation = tag.fieldOrientation.toOrientation(AxesReference.EXTRINSIC, AxesOrder.YXZ, AngleUnit.RADIANS);

        Point cameraVectorToTag = fieldVectorToTag.rotate(-cameraFieldAngle);
        double x = cameraVectorToTag.y; // Yes, swap 'x' and 'y'
        double y = cameraVectorToTag.x; // Yes, swap 'x' and 'y'
        double z = 0;

        // Add 90 degrees to the tag's angle to orient both angles in a consistent direction:
        double yaw = (tagOrientation.thirdAngle + Math.PI/2) - cameraFieldAngle;
        double pitch = 0;
        double roll = 0;
        double range = cameraVectorToTag.length();
        double bearing = Math.atan2(x, y); // Yes, 'x' is rise, 'y' is run
        double elevation = 0;

        double yawInDegrees = Math.toDegrees(yaw);
        double bearingInDegrees = Math.toDegrees(bearing);

        AprilTagPoseFtc ftcPose = new AprilTagPoseFtc(-x, y, z, yawInDegrees, pitch, roll, range, bearingInDegrees, elevation);

        return new AprilTagDetection(tag.id, 0, 0, null, null,
                tag, ftcPose, null, 0);
    }

    @Override
    public void setDecimation(float decimation) {}

    @Override
    public void setPoseSolver(PoseSolver poseSolver) {}

    @Override
    public int getPerTagAvgPoseSolveTime() {
        return 0;
    }

    @Override
    public ArrayList<AprilTagDetection> getFreshDetections() {
        if (cameraDescriptor == null)
            return null;

        // Cap the fresh detections to the maximum frame-rate:
        if (WilyCore.time() - lastDetectionTime < 1 / MAX_FPS)
            return null;

        // @@@ Check if enabled
        ArrayList<AprilTagDetection> detections = new ArrayList<>();
        Pose2d pose = WilyCore.getPose(cameraDescriptor.latency);
        for (AprilTagMetadata tag: tags) {
            double poseHeading = pose.heading.log();
            Point robotCameraOffset = new Point(cameraDescriptor.x, cameraDescriptor.y);
            Point fieldCameraOffset = robotCameraOffset.rotate(poseHeading);
            Point fieldCameraPosition = new Point(pose.position).add(fieldCameraOffset);

            Point vectorToTag = new Point(tag.fieldPosition).subtract(fieldCameraPosition);
            double tagAngle = vectorToTag.atan2();

            double cameraFieldAngle = poseHeading + cameraDescriptor.orientation;
            double halfFov = (cameraDescriptor.fieldOfView / 2) * 1.3;
            double rightAngle = cameraFieldAngle - halfFov;
            double leftAngle = cameraFieldAngle + halfFov;
            double deltaRight = Globals.normalizeAngle(tagAngle - rightAngle);
            double deltaLeft = Globals.normalizeAngle(leftAngle - tagAngle);
            if ((deltaRight > 0) && (deltaLeft > 0)) {
                detections.add(createDetection(tag, cameraFieldAngle, vectorToTag));
            }
        }

        lastDetections = detections;
        lastDetectionTime = WilyCore.time();
        return detections;
    }

    @Override
    public ArrayList<AprilTagDetection> getDetections() {
         ArrayList<AprilTagDetection> detections = getFreshDetections();
        if (detections == null)
            detections = lastDetections;
         return detections;
    }
}
