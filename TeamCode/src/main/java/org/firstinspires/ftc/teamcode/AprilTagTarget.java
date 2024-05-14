package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AprilTagTarget {
    public final int aprilTagId;
    public final FieldCoordinate aprilTagLocation;
    public final double validRange;
    public final double validAngleDeg;

    public AprilTagTarget(int tagId, FieldCoordinate location, double range, double angleDeg) {
        aprilTagId = tagId;
        aprilTagLocation = new FieldCoordinate(location);
        validRange = range;
        validAngleDeg = angleDeg;
    }

    // Determine if this is a good detection to generate a location from
    public boolean isReliableDetection(AprilTagDetection detection) {
        // Could we potentially change this to a more adaptive factor where the closer you
        // are the more angle is allowed?
        return ((Math.abs(detection.ftcPose.yaw) < validAngleDeg) &&
                (detection.ftcPose.range < validRange));
    }
    // Calculate a location based on an april tag detection
    public FieldCoordinate calculateCoordinatesFromDetection(AprilTagDetection detection) {
        FieldCoordinate result = null;
        if((detection != null) && (detection.ftcPose != null) && (isReliableDetection(detection))) {
            double myPositionX = detection.ftcPose.range * Math.cos(
                    Math.toRadians(aprilTagLocation.getAngleDegrees() + detection.ftcPose.bearing)) +
                    aprilTagLocation.getX();
            double myPositionY = detection.ftcPose.range * Math.sin(
                    Math.toRadians(aprilTagLocation.getAngleDegrees() + detection.ftcPose.bearing)) +
                    aprilTagLocation.getY();
            double myPositionAngleDegrees = aprilTagLocation.getAngleDegrees() - detection.ftcPose.yaw;
            result = new FieldCoordinate(myPositionX, myPositionY, myPositionAngleDegrees);
        }

        return result;
    }
}
