/* Copyright (c) 2022 FIRST. All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

/*
 * This OpMode shows how to use the new universal IMU interface. This
 * interface may be used with the BNO055 IMU or the BHI260 IMU. It assumes that an IMU is configured
 * on the robot with the name "imu".
 *
 * The sample will display the current Yaw, Pitch and Roll of the robot.<br>
 * With the correct orientation parameters selected, pitch/roll/yaw should act as follows:
 *   Pitch value should INCREASE as the robot is tipped UP at the front. (Rotation about X)
 *   Roll value should INCREASE as the robot is tipped UP at the left side. (Rotation about Y)
 *   Yaw value should INCREASE as the robot is rotated Counter Clockwise. (Rotation about Z)
 *
 * The yaw can be reset (to zero) by pressing the Y button on the gamepad (Triangle on a PS4 controller)
 *
 * This specific sample DOES NOT assume that the Hub is mounted on one of the three orthogonal
 * planes (X/Y, X/Z or Y/Z) OR that the Hub has only been rotated in a range of 90 degree increments.
 *
 * Note: if your Hub is mounted Orthogonally (on a orthogonal surface, angled at some multiple of
 * 90 Degrees) then you should use the simpler SensorImuOrthogonal sample in this folder.
 *
 * But... If your Hub is mounted Non-Orthogonally, you must specify one or more rotational angles
 * that transform a "Default" Hub orientation into your desired orientation.  That is what is
 * illustrated here.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 * Finally, edit this OpMode to use at least one angle around an axis to orient your Hub.
 */
@TeleOp(name = "Sensor: IMU Non-Orthogonal", group = "Sensor")
@Disabled     // Comment this out to add to the OpMode list
public class SensorIMUNonOrthogonal extends LinearOpMode
{
    // The IMU sensor object
    IMU imu;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() throws InterruptedException {

        // Retrieve and initialize the IMU.
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");

        /* Define how the hub is mounted to the robot to get the correct Yaw, Pitch and Roll values.
         *
         * You can apply up to three axis rotations to orient your Hub according to how it's mounted on the robot.
         *
         * The starting point for these rotations is the "Default" Hub orientation, which is:
         * 1) Hub laying flat on a horizontal surface, with the Printed Logo facing UP
         * 2) Rotated such that the USB ports are facing forward on the robot.
         *
         * The order that the rotations are performed matters, so this sample shows doing them in the order X, Y, then Z.
         * For specifying non-orthogonal hub mounting orientations, we must temporarily use axes
         * defined relative to the Hub itself, instead of the usual Robot Coordinate System axes
         * used for the results the IMU gives us. In the starting orientation, the Hub axes are
         * aligned with the Robot Coordinate System:
         *
         * X Axis:  Starting at Center of Hub, pointing out towards I2C connectors
         * Y Axis:  Starting at Center of Hub, pointing out towards USB connectors
         * Z Axis:  Starting at Center of Hub, pointing Up through LOGO
         *
         * Positive rotation is defined by right-hand rule with thumb pointing in +ve direction on axis.
         *
         * Some examples.
         *
         * ----------------------------------------------------------------------------------------------------------------------------------
         * Example A) Assume that the hub is mounted on a sloped plate at the back of the robot, with the USB ports coming out the top of the hub.
         *  The plate is tilted UP 60 degrees from horizontal.
         *
         *  To get the "Default" hub into this configuration you would just need a single rotation.
         *  1) Rotate the Hub +60 degrees around the X axis to tilt up the front edge.
         *  2) No rotation around the Y or Z axes.
         *
         *  So the X,Y,Z rotations would be 60,0,0
         *
         * ----------------------------------------------------------------------------------------------------------------------------------
         * Example B) Assume that the hub is laying flat on the chassis, but it has been twisted 30 degrees towards the right front wheel to make
         *  the USB cable accessible.
         *
         *  To get the "Default" hub into this configuration you would just need a single rotation, but around a different axis.
         *  1) No rotation around the X or Y axes.
         *  1) Rotate the Hub -30 degrees (Clockwise) around the Z axis, since a positive angle would be Counter Clockwise.
         *
         *  So the X,Y,Z rotations would be 0,0,-30
         *
         * ----------------------------------------------------------------------------------------------------------------------------------
         *  Example C) Assume that the hub is mounted on a vertical plate on the right side of the robot, with the Logo facing out, and the
         *  Hub rotated so that the USB ports are facing down 30 degrees towards the back wheels of the robot.
         *
         *  To get the "Default" hub into this configuration will require several rotations.
         *  1) Rotate the hub +90 degrees around the X axis to get it standing upright with the logo pointing backwards on the robot
         *  2) Next, rotate the hub +90 around the Y axis to get it facing to the right.
         *  3) Finally rotate the hub +120 degrees around the Z axis to take the USB ports from vertical to sloping down 30 degrees and
         *     facing towards the back of the robot.
         *
         *  So the X,Y,Z rotations would be 90,90,120
         */

        // The next three lines define the desired axis rotations.
        // To Do: EDIT these values to match YOUR mounting configuration.
        double xRotation = 0;  // enter the desired X rotation angle here.
        double yRotation = 0;  // enter the desired Y rotation angle here.
        double zRotation = 0;  // enter the desired Z rotation angle here.

        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);

        // Now initialize the IMU with this mounting orientation
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Loop and update the dashboard
        while (!isStopRequested()) {
            telemetry.addData("Hub orientation", "X=%.1f,  Y=%.1f,  Z=%.1f \n", xRotation, yRotation, zRotation);

            // Check to see if heading reset is requested
            if (gamepad1.y) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            } else {
                telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset\n");
            }

            // Retrieve Rotational Angles and Velocities
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.update();
        }
    }
}
