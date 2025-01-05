/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

public class PinpointBlocks extends BlocksOpModeCompanion {

    private static GoBildaPinpointDriver.DeviceStatus deviceStatus;
    private static int loopTime;
    private static int xEncoder;
    private static int yEncoder;
    private static double posX;
    private static double posY;
    private static double posH;
    private static double velX;
    private static double velY;
    private static double velH;

    private static final int GREEN = 147;
    private static final int PURPLE = 289;

    @ExportToBlocks(
            heading = "",
            comment = "ticks per unit of the goBILDA 4-Bar Odometry Pod",
            parameterDefaultValues = "MM",
            color = GREEN
    )
    public static double FourBarOdometryPod(DistanceUnit unit) {
        return unit.fromMm(19.89436789);
    }

    @ExportToBlocks(
            heading = "",
            comment = "ticks per unit of the goBILDA Swingarm Odometry Pod",
            parameterDefaultValues = "MM",
            color = GREEN
    )
    public static double SwingarmOdometryPod(DistanceUnit unit) {
        return unit.fromMm(13.26291192);
    }

    @ExportToBlocks(
            heading = "call",
            comment = "Call this once per loop to read new data from the Odometry Computer. Data will only update once this is called.",
            color = PURPLE
    )
    public static void update() {
        List<GoBildaPinpointDriver> pinpoints;
        pinpoints = hardwareMap.getAll(GoBildaPinpointDriver.class);
        if (!pinpoints.isEmpty()) {
            GoBildaPinpointDriver odo = pinpoints.get(0);
            odo.update();
            deviceStatus = odo.getDeviceStatus();
            loopTime = odo.getLoopTime();
            xEncoder = odo.getEncoderX();
            yEncoder = odo.getEncoderY();
            posX = odo.getPosX();
            posY = odo.getPosY();
            posH = odo.getHeading();
            velX = odo.getVelX();
            velY = odo.getVelY();
            velH = odo.getHeadingVelocity();
        }
    }

    @ExportToBlocks(
            heading = "call",
            comment = "Setting the Boolean to true reverses the encoder, false leaves it normal",
            parameterLabels = {"X Encoder Reversed?", "Y Encoder Reversed?"},
            parameterDefaultValues = {"false", "false"},
            color = PURPLE
    )
    public static void reverseEncoders(boolean xEncoder, boolean yEncoder) {
        GoBildaPinpointDriver.EncoderDirection xEn = null;
        GoBildaPinpointDriver.EncoderDirection yEn = null;

        List<GoBildaPinpointDriver> pinpoints;
        pinpoints = hardwareMap.getAll(GoBildaPinpointDriver.class);
        if (!pinpoints.isEmpty()) {
            GoBildaPinpointDriver odo = pinpoints.get(0);
            if (xEncoder) {
                xEn = GoBildaPinpointDriver.EncoderDirection.REVERSED;
            } else if (!xEncoder) {
                xEn = GoBildaPinpointDriver.EncoderDirection.FORWARD;
            }

            if (yEncoder) {
                yEn = GoBildaPinpointDriver.EncoderDirection.REVERSED;
            } else if (!yEncoder) {
                yEn = GoBildaPinpointDriver.EncoderDirection.FORWARD;
            }
            odo.setEncoderDirections(xEn, yEn);
        }
    }

    @ExportToBlocks(
            heading = "set",
            comment = "sets the number of ticks per mm of linear travel for the odometry pod you are using",
            tooltip = "you can calculate this number by dividing the counts-per-revolution of your encoder " +
                    "by the circumference of the wheel in specified unit.",
            parameterDefaultValues = {"13.26291192", "MM"},
            color = GREEN
    )
    public static void encoderResolution(double res, DistanceUnit unit) {
        List<GoBildaPinpointDriver> pinpoints;
        pinpoints = hardwareMap.getAll(GoBildaPinpointDriver.class);
        if (!pinpoints.isEmpty()) {
            GoBildaPinpointDriver odo = pinpoints.get(0);
            odo.setEncoderResolution(unit.toMm(res));
        }
    }

    @ExportToBlocks(
            heading = "set",
            comment = "Sets the odometry pod positions relative to the point that the odometry computer tracks around." +
                    "The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is. left is positive" +
                    "the Y Pod offset refers to how far forward from the tracking point the Y (strafe) odometry pod is. Forward increases",
            parameterLabels = {"Units to use:", "X Pod Offset:", "Y Pod Offset:"},
            parameterDefaultValues = {"MM", "-84", "-168"},
            color = GREEN
    )
    public static void offsets(DistanceUnit unit, double xOffset, double yOffset) {
        List<GoBildaPinpointDriver> pinpoints;
        pinpoints = hardwareMap.getAll(GoBildaPinpointDriver.class);
        if (!pinpoints.isEmpty()) {
            GoBildaPinpointDriver odo = pinpoints.get(0);
            odo.setOffsets(unit.toMm(xOffset), unit.toMm(yOffset));
        }
    }

    @ExportToBlocks(
            heading = "call",
            comment = "Resets the current position to 0,0,0 and recalibrates the Odometry Computer's internal IMU. " +
                    "Robot MUST Be stationary. Device takes a large number of samples, and uses those as the gyroscope zero-offset. " +
                    "This takes approximately 0.25 seconds.",
            color = PURPLE
    )
    public static void resetPosAndIMU() {
        List<GoBildaPinpointDriver> pinpoints;
        pinpoints = hardwareMap.getAll(GoBildaPinpointDriver.class);
        if (!pinpoints.isEmpty()) {
            GoBildaPinpointDriver odo = pinpoints.get(0);
            odo.resetPosAndIMU();
        }
    }

    @ExportToBlocks(
            heading = "call",
            comment = "Recalibrates the Odometry Computer's internal IMU. Robot MUST Be stationary. " +
                    "Device takes a large number of samples, and uses those as the gyroscope zero-offset. " +
                    "This takes approximately 0.25 seconds.",
            color = PURPLE
    )
    public static void recalibrateIMU() {
        List<GoBildaPinpointDriver> pinpoints;
        pinpoints = hardwareMap.getAll(GoBildaPinpointDriver.class);
        if (!pinpoints.isEmpty()) {
            GoBildaPinpointDriver odo = pinpoints.get(0);
            odo.recalibrateIMU();
        }
    }

    @ExportToBlocks(
            heading = "set",
            comment = "Tuning this value should be unnecessary. The goBILDA Odometry Computer has a " +
                    "per-device tuned yaw offset already applied when you receive it. " +
                    "This is a scalar that is applied to the gyro's yaw value. Increasing it will mean it " +
                    "will report more than one degree for every degree the sensor fusion algorithm measures. ",
            color = GREEN
    )
    public static void yawScalar(double yawScalar) {
        List<GoBildaPinpointDriver> pinpoints;
        pinpoints = hardwareMap.getAll(GoBildaPinpointDriver.class);
        if (!pinpoints.isEmpty()) {
            GoBildaPinpointDriver odo = pinpoints.get(0);
            odo.setYawScalar(yawScalar);
        }
    }

    @ExportToBlocks(
            heading = "set",
            comment = "Send a position that the Pinpoint should use to track your robot relative to. " +
                    "You can use this to update the estimated position of your robot with new external " +
                    "sensor data, or to run a robot in field coordinates. ",
            tooltip = "X/Y can be in mm, cm, inches, or even meters! Heading can be in degrees or radians ",
            parameterLabels = {"Unit for X/Y", "New X Position:", "New Y Position:", "Unit for Heading", "New Heading: "},
            parameterDefaultValues = {"MM", "0", "0", "DEGREES", "0"},
            color = GREEN
    )
    public static void position(DistanceUnit distanceUnit, double x, double y, AngleUnit angleUnit, double h) {
        List<GoBildaPinpointDriver> pinpoints;
        pinpoints = hardwareMap.getAll(GoBildaPinpointDriver.class);
        if (!pinpoints.isEmpty()) {
            GoBildaPinpointDriver odo = pinpoints.get(0);
            odo.setPosition(new Pose2D(distanceUnit, x, y, angleUnit, h));
        }
    }

    @ExportToBlocks(
            heading = "",
            comment = "Firmware version of the device. This requires it's own I²C read, so do not read it every loop.",
            color = GREEN
    )
    public static int deviceVersion() {
        List<GoBildaPinpointDriver> pinpoints;
        pinpoints = hardwareMap.getAll(GoBildaPinpointDriver.class);
        if (!pinpoints.isEmpty()) {
            GoBildaPinpointDriver odo = pinpoints.get(0);
            return odo.getDeviceVersion();
        } else {
            return 0;
        }
    }

    @ExportToBlocks(
            heading = "",
            comment = "Returns the current yawScalar set to the device (returns factory calibration if user has not applied one since power cycle).",
            color = GREEN
    )
    public static float yawScalar() {
        List<GoBildaPinpointDriver> pinpoints;
        pinpoints = hardwareMap.getAll(GoBildaPinpointDriver.class);
        if (!pinpoints.isEmpty()) {
            GoBildaPinpointDriver odo = pinpoints.get(0);
            return odo.getYawScalar();
        } else {
            return 0;
        }
    }

    @ExportToBlocks(
            heading = "",
            comment = "Returns the current status of the Pinpoint as a string. Can be one of the following: " +
                    "NOT_READY: powering up | READY: working correctly | CALIBRATING: Zero Offset is being measured " +
                    "| FAULT_NO_PODS_DETECTED: No odometry pods are plugged in | FAULT_X_POD_NOT_DETECTED: X pod missing " +
                    "| FAULT_Y_POD_NOT_DETECTED: Y Pod is missing ",
            color = GREEN
    )
    public static String deviceStatus() {
        if (deviceStatus == GoBildaPinpointDriver.DeviceStatus.READY) {
            return "READY";
        } else if (deviceStatus == GoBildaPinpointDriver.DeviceStatus.NOT_READY) {
            return "NOT_READY";
        } else if (deviceStatus == GoBildaPinpointDriver.DeviceStatus.CALIBRATING) {
            return "CALIBRATING";
        } else if (deviceStatus == GoBildaPinpointDriver.DeviceStatus.FAULT_NO_PODS_DETECTED) {
            return "FAULT_NO_PODS_DETECTED";
        } else if (deviceStatus == GoBildaPinpointDriver.DeviceStatus.FAULT_X_POD_NOT_DETECTED) {
            return "FAULT_X_POD_NOT_DETECTED";
        } else if (deviceStatus == GoBildaPinpointDriver.DeviceStatus.FAULT_Y_POD_NOT_DETECTED) {
            return "FAULT_Y_POD_NOT_DETECTED";
        } else {
            return "PINPOINT_NOT_DETECTED";
        }
    }

    @ExportToBlocks(
            heading = "",
            comment = "Checks the Odometry Computer's most recent loop time in microseconds",
            color = GREEN
    )
    public static int loopTime() {
        List<GoBildaPinpointDriver> pinpoints;
        pinpoints = hardwareMap.getAll(GoBildaPinpointDriver.class);
        if (!pinpoints.isEmpty()) {
            GoBildaPinpointDriver odo = pinpoints.get(0);
            return odo.getLoopTime();
        } else {
            return 0;
        }
    }

    @ExportToBlocks(
            heading = "",
            comment = "Checks the Odometry Computer's most recent loop frequency in hz (loops per second)",
            color = GREEN
    )
    public static double frequency() {
        List<GoBildaPinpointDriver> pinpoints;
        pinpoints = hardwareMap.getAll(GoBildaPinpointDriver.class);
        if (!pinpoints.isEmpty()) {
            GoBildaPinpointDriver odo = pinpoints.get(0);
            return odo.getFrequency();
        } else {
            return 0;
        }
    }

    @ExportToBlocks(
            heading = "",
            comment = "Returns x position the unit of your choice",
            tooltip = "You can get this in mm, inches, cm, and even meters!",
            parameterDefaultValues = {"MM"},
            color = GREEN
    )
    public static double xPosition(DistanceUnit unit) {
        return unit.fromMm(posX);
    }

    @ExportToBlocks(
            heading = "",
            comment = "Returns y position the unit of your choice",
            tooltip = "You can get this in mm, inches, cm, and even meters!",
            parameterDefaultValues = {"MM"},
            color = GREEN
    )
    public static double yPosition(DistanceUnit unit) {
        return unit.fromMm(posY);
    }

    @ExportToBlocks(
            heading = "",
            comment = "Returns the direction your robot is facing the unit of your choice",
            tooltip = "You can get this in radians or degrees!",
            parameterDefaultValues = {"DEGREES"},
            color = GREEN
    )
    public static double orientation(AngleUnit unit) {
        return unit.fromRadians(posH);
    }

    @ExportToBlocks(
            heading = "",
            comment = "Returns X robot velocity in units/second. So DistanceUnit.Inches becomes inches/second.",
            tooltip = "You can get this in mm/sec, inches/sec, cm/sec, and even meters/sec!",
            parameterDefaultValues = {"MM"},
            color = GREEN
    )
    public static double xVelocity(DistanceUnit unit) {
        return unit.fromMm(velX);
    }

    @ExportToBlocks(
            heading = "",
            comment = "Returns Y robot velocity in units/second. So DistanceUnit.Inches becomes inches/second.",
            tooltip = "You can get this in mm/sec, inches/sec, cm/sec, and even meters/sec!",
            parameterDefaultValues = {"MM"},
            color = GREEN
    )
    public static double yVelocity(DistanceUnit unit) {
        return unit.fromMm(velY);
    }

    @ExportToBlocks(
            heading = "",
            comment = "Returns heading velocity in units/second. So AngleUnit.degrees becomes degrees/second.",
            tooltip = "You can get this in radians/sec or degrees/sec!",
            parameterDefaultValues = {"DEGREES"},
            color = GREEN
    )
    public static double headingVelocity(AngleUnit unit) {
        return unit.fromRadians(velH);
    }

    @ExportToBlocks(
            heading = "",
            comment = "Returns the raw encoder value of the X Encoder. This is not changed when the user" +
                    " setEncoderDirection, and is not reset with Pinpoint functions.",
            color = GREEN
    )
    public static int xEncoder() {
        return xEncoder;
    }

    @ExportToBlocks(
            heading = "",
            comment = "Returns the raw encoder value of the Y Encoder. This is not changed when the user" +
                    " setEncoderDirection, and is not reset with Pinpoint functions.",
            color = GREEN
    )
    public static int yEncoder() {
        return yEncoder;
    }

}
