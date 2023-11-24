package org.firstinspires.ftc.teamcode.types;

public class HydraAprilTagPose {
    /**
     * read this to understand these values
     * <a href="https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_reference_frame/apriltag-reference-frame.html">...</a>
     */
    public int mId;
    // left-right distance from the center point of the frame to the center of the april tag
    public double mPoseX;
    // distance to the plane containing the center point of the frame. As far as I can tell this is
    // not the distance to the tag, but the distance to that plane that is parallel to the camera's view
    public double mPoseY;
    // vertical distance from the center
    public double mPoseZ;
    // this is the angle up or down around the X axis
    public double mPosePitch;
    // this is the rotation around the Y axis if the tag is tilted compared to the camera
    public double mPoseRoll;
    // rotation around the Z axis, e.g. a left or right displaced tag that directly faces the camera
    public double mPoseYaw;
    // point to point distance to the tag center
    public double mPoseRange;
    // angle the camera must turn to point directly to the tag center
    public double mPoseBearing;
    // angle the camera must tilt up or down to point at the tag center
    public double mPoseElevation;
}
