/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of calibrating a MR Compass
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 *   This code assumes there is a compass configured with the name "compass"
 *
 *   This code will put the compass into calibration mode, wait three seconds and then attempt
 *   to rotate two full turns clockwise.  This will allow the compass to do a magnetic calibration.
 *
 *   Once compete, the program will put the compass back into measurement mode and check to see if the
 *   calibration was successful.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Concept: Compass Calibration", group="Concept")
@Disabled
public class ConceptCompassCalibration extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot     robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    CompassSensor       compass;

    final static double     MOTOR_POWER   = 0.2; // scale from 0 to 1
    static final long       HOLD_TIME_MS  = 3000;
    static final double     CAL_TIME_SEC  = 20;

    @Override
    public void runOpMode() {

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // get a reference to our Compass Sensor object.
        compass = hardwareMap.get(CompassSensor.class, "compass");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to cal");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Set the compass to calibration mode
        compass.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);
        telemetry.addData("Compass", "Compass in calibration mode");
        telemetry.update();

        sleep(HOLD_TIME_MS);  // Just do a sleep while we switch modes

        // Start the robot rotating clockwise
        telemetry.addData("Compass", "Calibration mode. Turning the robot...");
        telemetry.update();
        robot.leftDrive.setPower(MOTOR_POWER);
        robot.rightDrive.setPower(-MOTOR_POWER);

        // run until time expires OR the driver presses STOP;
        runtime.reset();
        while (opModeIsActive() && (runtime.time() < CAL_TIME_SEC)) {
            idle();
        }

        // Stop all motors and turn off claibration
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        compass.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);
        telemetry.addData("Compass", "Returning to measurement mode");
        telemetry.update();

        sleep(HOLD_TIME_MS);  // Just do a sleep while we switch modes

        // Report whether the Calibration was successful or not.
        if (compass.calibrationFailed())
            telemetry.addData("Compass", "Calibrate Failed. Try Again!");
        else
            telemetry.addData("Compass", "Calibrate Passed.");
        telemetry.update();
    }
}
