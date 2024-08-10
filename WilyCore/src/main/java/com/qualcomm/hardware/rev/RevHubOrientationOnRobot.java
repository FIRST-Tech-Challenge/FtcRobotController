/*
Copyright (c) 2022 REV Robotics

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of REV Robotics nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package com.qualcomm.hardware.rev;

import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.QuaternionBasedImuHelper;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

/**
 * The orientation at which a given REV Robotics Control Hub or Expansion Hub is mounted to a robot.
 * <p>
 * Pass to the {@link com.qualcomm.robotcore.hardware.IMU.Parameters} constructor.
 */
public class RevHubOrientationOnRobot implements ImuOrientationOnRobot {
    public enum LogoFacingDirection {
        UP,
        DOWN,
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    public enum UsbFacingDirection {
        UP,
        DOWN,
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    private final Quaternion robotCoordinateSystemFromPerspectiveOfImu;
    private final Quaternion imuRotationOffset;
    private final Quaternion angularVelocityTransform;

    /**
     * Constructs a {@link RevHubOrientationOnRobot} for a REV Hub that is mounted orthogonally to a
     * robot. This is the easiest constructor to use. Simply specify which direction on the robot
     * that the REV Robotics logo on the Hub is facing, and the direction that the USB port(s) on
     * the Hub are facing.
     * @param logoFacingDirection The direction that the REV Robotics logo is facing on the robot.
     * @param usbFacingDirection The direction that the USB port(s) are facing on the robot.
     */
    public RevHubOrientationOnRobot(LogoFacingDirection logoFacingDirection, UsbFacingDirection usbFacingDirection) {
        this(friendlyApiToOrientation(logoFacingDirection, usbFacingDirection));
    }

    /**
     * Constructs a {@link RevHubOrientationOnRobot} for a REV Hub that is mounted at any arbitrary
     * angle on a robot using an {@link Orientation} object.
     * @param rotation The rotation (defined within the Robot Coordinate System, and specified as an
     *                 {@link Orientation}) that would need to be applied in order to rotate a REV
     *                 Control or Expansion Hub from having its logo facing up and the USB ports
     *                 facing forward, to its actual orientation on the robot.
     */
    public RevHubOrientationOnRobot(Orientation rotation) {
        this(Quaternion.fromMatrix(rotation.getRotationMatrix(), 0));
    }

    /**
     * Constructs a {@link RevHubOrientationOnRobot} for a REV Hub that is mounted at any arbitrary
     * angle on a robot using a {@link Quaternion} object.
     * @param rotation The rotation (defined within the Robot Coordinate System, and specified as a
     *                 {@link Quaternion}) that would need to be applied in order to rotate a REV
     *                 Control or Expansion Hub from having its logo facing up and the USB ports
     *                 facing forward, to its actual orientation on the robot.
     */
    public RevHubOrientationOnRobot(Quaternion rotation) {
        // When an Expansion Hub or Control Hub has its logo facing up and the USB ports facing
        // forward, the IMU's axes are rotated by -90 degrees around the Z axis compared to the
        // Robot Coordinate System's axes, so we need to apply that rotation to whatever the user
        // has specified.
        Quaternion imuRotationWithinHub = QuaternionBasedImuHelper.quaternionFromZAxisRotation(-90, AngleUnit.DEGREES);
        rotation = rotation.multiply(imuRotationWithinHub, 0).normalized();

        // A rotation that will take the hub from the "default" orientation to its actual
        // orientation when applied in the Robot Coordinate System, will take the hub from its
        // actual orientation to the "default" orientation when applied in the IMU's coordinate
        // system, which is what the angular velocity transform is!
        this.angularVelocityTransform = new Quaternion(rotation.w, rotation.x, rotation.y, rotation.z, 0);

        // For the BNO055 and BHI260, the Z axis always points upward, just like in the Robot
        // Coordinate System, regardless of the starting orientation (this is accomplished using
        // their internal accelerometers). This means that the only axes that ever need to be
        // remapped are the X and Y axes, which we can do with the Z axis component of the rotation.
        this.robotCoordinateSystemFromPerspectiveOfImu = new Quaternion(rotation.w, 0, 0, rotation.z, 0).normalized();

        // The Z axis rotation will always start out at zero, so we don't want to include that in
        // the rotation offset.
        rotation = rotation.multiply(robotCoordinateSystemFromPerspectiveOfImu.inverse(), 0).normalized();

        // The rotation was provided as the rotation that needs to be applied to a hub in the
        // "default" orientation to get it to its actual orientation. The offset that will be
        // applied needs to be the opposite (inverse) of that.
        this.imuRotationOffset = rotation.inverse().normalized();
    }

    @Override public Quaternion imuCoordinateSystemOrientationFromPerspectiveOfRobot() {
        return robotCoordinateSystemFromPerspectiveOfImu;
    }

    @Override public Quaternion imuRotationOffset() {
        return imuRotationOffset;
    }

    @Override public Quaternion angularVelocityTransform() {
        return angularVelocityTransform;
    }

    protected static Orientation friendlyApiToOrientation(LogoFacingDirection logoFacingDirection, UsbFacingDirection usbFacingDirection) {
        if (logoFacingDirection == LogoFacingDirection.UP) {
            if (usbFacingDirection == UsbFacingDirection.UP) {
                throwIllegalHubOrientationException();
            } else if (usbFacingDirection == UsbFacingDirection.DOWN) {
                throwIllegalHubOrientationException();
            } else if (usbFacingDirection == UsbFacingDirection.FORWARD) {
                return zyxOrientation(0, 0, 0);
            } else if (usbFacingDirection == UsbFacingDirection.BACKWARD) {
                return zyxOrientation(180, 0, 0);
            } else if (usbFacingDirection == UsbFacingDirection.LEFT) {
                return zyxOrientation(90, 0, 0);
            } else if (usbFacingDirection == UsbFacingDirection.RIGHT) {
                return zyxOrientation(-90, 0, 0);
            }
        } else if (logoFacingDirection == LogoFacingDirection.DOWN) {
            if (usbFacingDirection == UsbFacingDirection.UP) {
                throwIllegalHubOrientationException();
            } else if (usbFacingDirection == UsbFacingDirection.DOWN) {
                throwIllegalHubOrientationException();
            } else if (usbFacingDirection == UsbFacingDirection.FORWARD) {
                return zyxOrientation(0, 180, 0);
            } else if (usbFacingDirection == UsbFacingDirection.BACKWARD) {
                return zyxOrientation(180, 180, 0);
            } else if (usbFacingDirection == UsbFacingDirection.LEFT) {
                return zyxOrientation(90, 180, 0);
            } else if (usbFacingDirection == UsbFacingDirection.RIGHT) {
                return zyxOrientation(-90, 180, 0);
            }
        } else if (logoFacingDirection == LogoFacingDirection.FORWARD) {
            if (usbFacingDirection == UsbFacingDirection.UP) {
                return xyzOrientation(90, 180, 0);
            } else if (usbFacingDirection == UsbFacingDirection.DOWN) {
                return xyzOrientation(-90, 0, 0);
            } else if (usbFacingDirection == UsbFacingDirection.FORWARD) {
                throwIllegalHubOrientationException();
            } else if (usbFacingDirection == UsbFacingDirection.BACKWARD) {
                throwIllegalHubOrientationException();
            } else if (usbFacingDirection == UsbFacingDirection.LEFT) {
                return xyzOrientation(-90, 0, 90);
            } else if (usbFacingDirection == UsbFacingDirection.RIGHT) {
                return xyzOrientation(-90, 0, -90);
            }
        } else if (logoFacingDirection == LogoFacingDirection.BACKWARD) {
            if (usbFacingDirection == UsbFacingDirection.UP) {
                return xyzOrientation(90, 0, 0);
            } else if (usbFacingDirection == UsbFacingDirection.DOWN) {
                return xyzOrientation(90, 0, 180);
            } else if (usbFacingDirection == UsbFacingDirection.FORWARD) {
                throwIllegalHubOrientationException();
            } else if (usbFacingDirection == UsbFacingDirection.BACKWARD) {
                throwIllegalHubOrientationException();
            } else if (usbFacingDirection == UsbFacingDirection.LEFT) {
                return xyzOrientation(90, 0, 90);
            } else if (usbFacingDirection == UsbFacingDirection.RIGHT) {
                return xyzOrientation(90, 0, -90);
            }
        } else if (logoFacingDirection == LogoFacingDirection.LEFT) {
            if (usbFacingDirection == UsbFacingDirection.UP) {
                return xyzOrientation(90, -90, 0);
            } else if (usbFacingDirection == UsbFacingDirection.DOWN) {
                return xyzOrientation(-90, -90, 0);
            } else if (usbFacingDirection == UsbFacingDirection.FORWARD) {
                return xyzOrientation(0, -90, 0);
            } else if (usbFacingDirection == UsbFacingDirection.BACKWARD) {
                return xyzOrientation(0, -90, 180);
            } else if (usbFacingDirection == UsbFacingDirection.LEFT) {
                throwIllegalHubOrientationException();
            } else if (usbFacingDirection == UsbFacingDirection.RIGHT) {
                throwIllegalHubOrientationException();
            }
        } else if (logoFacingDirection == LogoFacingDirection.RIGHT) {
            if (usbFacingDirection == UsbFacingDirection.UP) {
                return zyxOrientation(90, 0, 90);
            } else if (usbFacingDirection == UsbFacingDirection.DOWN) {
                return zyxOrientation(-90, 0, -90);
            } else if (usbFacingDirection == UsbFacingDirection.FORWARD) {
                return zyxOrientation(0, 90, 0);
            } else if (usbFacingDirection == UsbFacingDirection.BACKWARD) {
                return zyxOrientation(180, -90, 0);
            } else if (usbFacingDirection == UsbFacingDirection.LEFT) {
                throwIllegalHubOrientationException();
            } else if (usbFacingDirection == UsbFacingDirection.RIGHT) {
                throwIllegalHubOrientationException();
            }
        }

        throw new RuntimeException("The FTC SDK developers forgot about this combination, please file a bug report.");
    }

    private static void throwIllegalHubOrientationException() {
        throw new IllegalArgumentException("The specified REV Hub orientation is physically impossible");
    }

    public static Orientation zyxOrientation(double z, double y, double x) {
        return new Orientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES, (float) z, (float) y, (float) x, 0);
    }

    public static Orientation xyzOrientation(double x, double y, double z) {
        return new Orientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, (float) x, (float) y, (float) z, 0);
    }
}
