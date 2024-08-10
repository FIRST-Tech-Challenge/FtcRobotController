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

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.function.ThrowingSupplier;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class QuaternionBasedImuHelper {
    private ImuOrientationOnRobot imuOrientationOnRobot;
    private Quaternion yawOffsetQuaternion = Quaternion.identityQuaternion();
    private Quaternion lastRobotOrientationWithoutYawOffset = Quaternion.identityQuaternion();

    public static Quaternion quaternionFromZAxisRotation(float rotation, AngleUnit angleUnit) {
        float rotationRadians = angleUnit.toRadians(rotation);
        return new Quaternion(
                (float) Math.cos(0.5 * rotationRadians),
                0,
                0,
                (float) Math.sin(0.5 * rotationRadians),
                0);
    }

    public QuaternionBasedImuHelper(ImuOrientationOnRobot imuOrientationOnRobot) {
        setImuOrientationOnRobot(imuOrientationOnRobot);
    }

    // If at all possible, we want to get an up-to-date quaternion when resetting the yaw, so we
    // keep trying until we reach a timeout, at which time we give up and just use the most
    // up-to-date data we have (which may be arbitrarily stale).
    public synchronized void resetYaw(
            String tag,
            ThrowingSupplier<Quaternion, FailedToRetrieveQuaternionException> imuCentricOrientationSupplier,
            int timeoutMs) {

        ElapsedTime timer = new ElapsedTime();
        Quaternion robotOrientationWithoutYawOffset = null;
        do {
            try {
                robotOrientationWithoutYawOffset = getRobotOrientationAsQuaternionOrThrow(imuCentricOrientationSupplier, false);
                break;
            } catch (FailedToRetrieveQuaternionException ignored) { }
        } while (timer.milliseconds() < timeoutMs);

        if (robotOrientationWithoutYawOffset == null) {
            // ### RobotLog.ww(tag, "Failed to retrieve valid quaternion for resetYaw(). Using stale orientation to compute yaw offset.");
            robotOrientationWithoutYawOffset = lastRobotOrientationWithoutYawOffset;
        }

        // Use the negated Z component of the quaternion
        yawOffsetQuaternion = new Quaternion(robotOrientationWithoutYawOffset.w, 0, 0, -robotOrientationWithoutYawOffset.z, 0).normalized();
    }

    public synchronized Quaternion getRobotOrientationAsQuaternionOrThrow(
            ThrowingSupplier<Quaternion, FailedToRetrieveQuaternionException> imuCentricOrientationSupplier,
            boolean applyYawOffset) throws FailedToRetrieveQuaternionException {

        Quaternion imuCentricOrientation = imuCentricOrientationSupplier.get();
        Quaternion rcsFromImuPerspective = imuOrientationOnRobot.imuCoordinateSystemOrientationFromPerspectiveOfRobot();

        long acquisitionTime = imuCentricOrientation.acquisitionTime;

        Quaternion originalOrientationInRobotCoordinateSystem = rcsFromImuPerspective
                .multiply(imuCentricOrientation.multiply(rcsFromImuPerspective.inverse(), acquisitionTime).normalized(), acquisitionTime).normalized();

        Quaternion result = originalOrientationInRobotCoordinateSystem.multiply(imuOrientationOnRobot.imuRotationOffset(), acquisitionTime).normalized();

        // Copy the result before we apply the yaw offset
        lastRobotOrientationWithoutYawOffset = new Quaternion(result.w, result.x, result.y, result.z, result.acquisitionTime);

        if (applyYawOffset) {
            result = yawOffsetQuaternion.multiply(result, acquisitionTime);
        }

        return result;
    }

    public synchronized Quaternion getRobotOrientationAsQuaternion(
            String tag,
            ThrowingSupplier<Quaternion, FailedToRetrieveQuaternionException> imuCentricOrientationSupplier,
            boolean applyYawOffset) {
        try {
            return getRobotOrientationAsQuaternionOrThrow(imuCentricOrientationSupplier, applyYawOffset);
        } catch (FailedToRetrieveQuaternionException e) {
            // ### RobotLog.ww(tag, "getRobotOrientationAsQuaternion(): Failed to retrieve valid quaternion from IMU. Returning the identity quaternion.");
            return Quaternion.identityQuaternion();
        }
    }

    public synchronized YawPitchRollAngles getRobotYawPitchRollAngles(
            String tag,
            ThrowingSupplier<Quaternion, FailedToRetrieveQuaternionException> imuCentricOrientationSupplier) {

        Orientation robotOrientation = getRobotOrientation(tag, imuCentricOrientationSupplier, AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        return new YawPitchRollAngles(
                AngleUnit.DEGREES,
                robotOrientation.firstAngle,
                robotOrientation.secondAngle,
                robotOrientation.thirdAngle,
                robotOrientation.acquisitionTime);
    }

    public synchronized Orientation getRobotOrientation(
            String tag,
            ThrowingSupplier<Quaternion, FailedToRetrieveQuaternionException> imuCentricOrientationSupplier,
            AxesReference reference,
            AxesOrder order,
            AngleUnit angleUnit) {
        return getRobotOrientationAsQuaternion(tag, imuCentricOrientationSupplier, true).toOrientation(reference, order, angleUnit);
    }

    public synchronized AngularVelocity getRobotAngularVelocity(AngularVelocity rawAngularVelocity, AngleUnit angleUnit) {
        rawAngularVelocity =  rawAngularVelocity.toAngleUnit(angleUnit);

        // Angular velocity can be thought of as a pseudo-vector, and we can adjust that
        // pseudo-vector by a quaternion that adapts the IMU's axes onto the robot's axes.
        VectorF rawAngularVelocityVector = new VectorF(
                rawAngularVelocity.xRotationRate,
                rawAngularVelocity.yRotationRate,
                rawAngularVelocity.zRotationRate);

        VectorF transformedAngularVelocityVector = imuOrientationOnRobot.angularVelocityTransform()
                .applyToVector(rawAngularVelocityVector);

        return new AngularVelocity(
                angleUnit,
                transformedAngularVelocityVector.get(0),
                transformedAngularVelocityVector.get(1),
                transformedAngularVelocityVector.get(2),
                rawAngularVelocity.acquisitionTime);
    }

    public synchronized void setImuOrientationOnRobot(ImuOrientationOnRobot imuOrientationOnRobot) {
        this.imuOrientationOnRobot = imuOrientationOnRobot;
    }

    public static class FailedToRetrieveQuaternionException extends Exception {}
}
