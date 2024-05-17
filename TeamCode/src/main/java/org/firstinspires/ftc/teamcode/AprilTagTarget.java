package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AprilTagTarget {
    public final int aprilTagId;
    public final FieldCoordinate aprilTagLocation;
    public final double validRange;
    public final double validAngleDeg;
    public final double lineSlope;
    public final double lineOffset;

    public AprilTagTarget(int tagId, FieldCoordinate location, double range, double angleDeg) {
        aprilTagId = tagId;
        aprilTagLocation = new FieldCoordinate(location);
        validRange = range;
        validAngleDeg = angleDeg;
        lineSlope = (0 - range) / (validAngleDeg - 0);
        lineOffset = validRange;
    }

    // Determine if this is a good detection to generate a location from
    public boolean isReliableDetection(AprilTagDetection detection) {
        return ((Math.abs(detection.ftcPose.yaw) < validAngleDeg) &&
                (detection.ftcPose.range < validRange));
    }
    // Determine if this is a good detection to generate a location from that
    // degrades range as angle increases up to validAngleDeg, where it is zeroed
    public boolean isReliableDetectionAdaptive(AprilTagDetection detection) {
        double angleBasedRange = lineSlope * Math.abs(detection.ftcPose.yaw) + lineOffset;
        return (detection.ftcPose.range < angleBasedRange);
    }
    // Calculate a location based on an april tag detection
    public FieldCoordinate calculateCoordinatesFromDetection(AprilTagDetection detection) {
        FieldCoordinate result = null;
        if((detection != null) && (detection.ftcPose != null) && (isReliableDetectionAdaptive(detection))) {
            double detectionAngleRadians = Math.toRadians(aprilTagLocation.getAngleDegrees() + detection.ftcPose.bearing);
            double myPositionX = detection.ftcPose.range * Math.cos(detectionAngleRadians) + aprilTagLocation.getX();
            double myPositionY = detection.ftcPose.range * Math.sin(detectionAngleRadians) + aprilTagLocation.getY();
            // This angle is assuming the camera is on the back of the robot.
            double myPositionAngleDegrees = aprilTagLocation.getAngleDegrees() - detection.ftcPose.yaw;
            result = new FieldCoordinate(myPositionX, myPositionY, myPositionAngleDegrees);
        }

        return result;
    }
}
