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

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

/**
 * Defines how an IMU is oriented relative to the robot. See
 * {@code com.qualcomm.hardware.rev.RevHubOrientationOnRobot} (from the Hardware module)
 * for an easy-to-use implementation for the REV Control and Expansion Hub.
 * <p>
 * Passed to the {@link IMU.Parameters} constructor.
 */
public interface ImuOrientationOnRobot {
    /**
     * @return The normalized quaternion (defined in the Robot Coordinate System) that represents
     *         how the IMU's coordinate system is oriented, given how it is mounted on the robot.
     *         This is used to convert results from the IMU's coordinate system into the Robot
     *         Coordinate System. Keep in mind that the IMU should keep its axes oriented based on
     *         gravity, so merely rotating the sensor around the robot's X or Y axis will not affect
     *         this value.
     */
    Quaternion imuCoordinateSystemOrientationFromPerspectiveOfRobot();

    /**
     * @return The normalized quaternion (defined in the Robot Coordinate System) that specifies the
     *         rotation that needs to be applied to the IMU's output (after it has been converted to
     *         the Robot Coordinate System) in order to in order to have the rotation around the
     *         robot X and Y axes be 0 when the robot is flat on the ground.
     */
    Quaternion imuRotationOffset();

    /**
     * @return The normalized quaternion (defined in the IMU's coordinate system) that specifies the
     *         rotation that needs to be applied to the IMU's angular velocity pseudo-vector to
     *         align it with the Robot Coordinate System.
     */
    Quaternion angularVelocityTransform();
}
