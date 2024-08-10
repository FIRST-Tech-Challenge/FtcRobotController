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
package com.qualcomm.robotcore.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * An Inertial Measurement Unit that provides robot-centric orientation and angular velocity.
 * <p>
 * All measurements are in the Robot Coordinate System. In the Robot Coordinate System, the X axis
 * extends horizontally from your robot to the right, parallel to the ground. The Y axis extends
 * horizontally from your robot straight ahead, parallel to the ground. The Z axis extends
 * vertically from your robot, towards the ceiling.
 * <p>
 * The Robot Coordinate System is right-handed, which means that if you point the thumb of a typical
 * human right hand in the direction of an axis, rotation around that axis is defined as positive in
 * the direction that the fingers curl.
 * <p>
 * Orientation values are relative to the robot's position the last time that {@link #resetYaw()}
 * was called, as if the robot was perfectly level at that time.
 * <p>
 * The recommended way to read the orientation is as yaw, pitch, and roll angles, via
 * {@link #getRobotYawPitchRollAngles()}. See the {@link YawPitchRollAngles} documentation for a
 * full description of how the angles get applied to each other. That class's documentation will
 * duplicate some information found here, but will not cover how yaw, pitch, and roll work in the
 * specific context of an IMU that implements this interface.
 * <p>
 * Yaw is the side-to-side lateral rotation of the robot. In terms of the Robot Coordinate
 * System, it is defined as how far the robot has turned around the Z axis. Sometimes yaw is also
 * referred to as "heading". The yaw can drift slowly over time, because most implementations do not
 * use a magnetometer as an absolute reference (magnetometer readings are disrupted by nearby
 * motors). The yaw reference is <i>preserved between OpMode runs</i>, unless the Robot Controller
 * application is restarted, the Restart Robot option is selected, or the second OpMode calls
 * {@link #resetYaw()}. This means that your yaw reference point can remain consistent through both
 * the Autonomous and TeleOp phases of the match.
 * <p>
 * Pitch is the front-to-back rotation of the robot. In terms of the Robot Coordinate System, it is
 * how far the robot has turned around the X axis. Pitch uses gravity as an absolute reference, and
 * will not drift over time.
 * <p>
 * Roll is the side-to-side tilt of the robot. In terms of the Robot Coordinate System, it is
 * defined as how far the robot has turned around the Y axis. Roll uses gravity as an absolute
 * reference, and will not drift over time.
 * <p>
 * All angles are in the range of -180 degrees to 180 degrees.
 * <p>
 * The default orientation of the IMU on the robot will be implementation-specific. For the BNO055
 * and BHI260AP implementations, the default is for a REV Control or Expansion Hub that is oriented
 * with the USB ports facing the front of the robot and the REV logo facing the ceiling. To specify
 * a non-default orientation on the robot, you need to call {@link #initialize(Parameters)}.
 */
public interface IMU extends HardwareDevice {
    /**
     * Settings to change the IMU's behavior. Used as the parameter of {@link #initialize(Parameters)}.
     * <p>
     * Some {@link IMU} implementations may have their own version of this class that adds
     * additional hardware-specific parameters.
     */
    class Parameters {
        // NOTE: Whenever this class is updated, you must also update the copy() method, and any
        // subclasses.
        public ImuOrientationOnRobot imuOrientationOnRobot;

        /**
         * @param imuOrientationOnRobot The orientation of the IMU relative to the robot. If the IMU
         *                              is in a REV Control or Expansion Hub, create an instance of
         *                              {@code com.qualcomm.hardware.rev.RevHubOrientationOnRobot}
         *                              (from the Hardware module).
         */
        public Parameters(ImuOrientationOnRobot imuOrientationOnRobot) {
            this.imuOrientationOnRobot = imuOrientationOnRobot;
        }

        public Parameters copy() {
            return new Parameters(imuOrientationOnRobot);
        }
    }

    /**
     * Initializes the IMU with non-default settings.
     *
     * @return Whether initialization succeeded.
     */
    boolean initialize(Parameters parameters);

    /**
     * Resets the robot's yaw angle to 0. After calling this method, the reported orientation will
     * be relative to the robot's position when this method was called, as if the robot was perfectly
     * level right then. That is to say, the pitch and yaw will be ignored when this method is
     * called.
     * <p>
     * Unlike yaw, pitch and roll are always relative to gravity, and never need to be reset.
     */
    void resetYaw();

    /**
     * @return A {@link YawPitchRollAngles} object representing the current orientation of the robot
     *         relative to the robot's position the last time that {@link #resetYaw()} was called,
     *         as if the robot was perfectly level at that time.
     */
    YawPitchRollAngles getRobotYawPitchRollAngles();

    /**
     * @return An {@link Orientation} object representing the current orientation of the robot
     *         relative to the robot's position the last time that {@link #resetYaw()} was called,
     *         as if the robot was perfectly level at that time.
     *         <p><p>
     *         The {@link Orientation} class provides many ways to represent the robot's orientation,
     *         which is helpful for advanced use cases. Most teams should use {@link #getRobotYawPitchRollAngles()}.
     */
    Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit);

    /**
     * @return A {@link Quaternion} object representing the current orientation of the robot
     *         relative to the robot's position the last time that {@link #resetYaw()} was called,
     *         as if the robot was perfectly level at that time.
     *         <p><p>
     *         Quaternions provide an advanced way to access orientation data that will work well
     *         for any orientation of the robot, even where other types of orientation data would
     *         encounter issues such as gimbal lock.
     */
    Quaternion getRobotOrientationAsQuaternion();

    /**
     * @return The angular velocity of the robot (how fast it's turning around the three axes).
     */
    AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit);
}
