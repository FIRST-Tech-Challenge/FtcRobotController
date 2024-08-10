/*
 * Copyright (c) 2023 FIRST
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.vision.apriltag;

/**
 * AprilTagPoseFtc represents the AprilTag's position in space, relative to the camera.
 * It is a realignment of the raw AprilTag Pose to be consistent with a forward-looking camera on an FTC robot.<BR>
 * Also includes additional derived values to simplify driving towards any Tag.<P>
 *
 * Note: These member definitions describe the camera Lens as the reference point for axis measurements.<BR>
 * This measurement may be off by the distance from the lens to the image sensor itself.  For most webcams this is a reasonable approximation.
 *
 * @see <a href="https://ftc-docs.firstinspires.org/apriltag-detection-values">apriltag-detection-values.pdf</a>
 */
public class AprilTagPoseFtc
{

    /**
     * X translation of AprilTag, relative to camera lens.  Measured sideways (Horizontally in camera image) the positive X axis extends out to the right of the camera viewpoint. <BR>
     * An x value of zero implies that the Tag is centered between the left and right sides of the Camera image.
     */
    public final double x;

    /**
     * Y translation of AprilTag, relative to camera lens.  Measured forwards (Horizontally in camera image) the positive Y axis extends out in the direction the camera is pointing.<BR>
     * A y value of zero implies that the Tag is touching (aligned with) the lens of the camera, which is physically unlikley.  This value should always be positive.  <BR>
     */
    public final double y;

    /**
     * Z translation of AprilTag, relative to camera lens.  Measured upwards (Vertically in camera image) the positive Z axis extends Upwards in the camera viewpoint.<BR>
     * A z value of zero implies that the Tag is centered between the top and bottom of the camera image.
     */
    public final double z;

    /**
     * Rotation of AprilTag around the Z axis.  Right-Hand-Rule defines positive Yaw rotation as Counter-Clockwise when viewed from above.<BR>
     * A yaw value of zero implies that the camera is directly in front of the Tag, as viewed from above.
     */
    public final double yaw;

    /**
     * Rotation of AprilTag around the X axis.  Right-Hand-Rule defines positive Pitch rotation as the Tag Image face twisting down when viewed from the camera.<BR>
     * A pitch value of zero implies that the camera is directly in front of the Tag, as viewed from the side.
     */
    public final double pitch;

    /**
     * Rotation of AprilTag around the Y axis.  Right-Hand-Rule defines positive Roll rotation as the Tag Image rotating Clockwise when viewed from the camera.<BR>
     * A roll value of zero implies that the Tag image is alligned squarely and upright, when viewed in the camera image frame.
     */
    public final double roll;

    /**
     * Range, (Distance), from the Camera lens to the center of the Tag, as measured along the X-Y plane (across the ground).
     */
    public final double range;

    /**
     * Bearing, or Horizontal Angle, from the "camera center-line", to the "line joining the Camera lens and the Center of the Tag".  <BR>
     * This angle is measured across the X-Y plane (across the ground).<BR>
     * A positive Bearing indicates that the robot must employ a positive Yaw (rotate counter clockwise) in order to point towards the target.
     */
    public final double bearing;

    /**
     * Elevation, (Vertical Angle), from "the camera center-line", to "the line joining the Camera Lens and the Center of the Tag".<BR>
     * A positive Elevation indicates that the robot must employ a positive Pitch (tilt up) in order to point towards the target.
     */
    public final double elevation;

    public AprilTagPoseFtc(double x, double y, double z, double yaw, double pitch,
                           double roll, double range, double bearing, double elevation)
    {
        this.x = x;
        this.y = y;
        this.z = z;
        this.yaw = yaw;
        this.pitch = pitch;
        this.roll = roll;
        this.range = range;
        this.bearing = bearing;
        this.elevation = elevation;
    }
}
