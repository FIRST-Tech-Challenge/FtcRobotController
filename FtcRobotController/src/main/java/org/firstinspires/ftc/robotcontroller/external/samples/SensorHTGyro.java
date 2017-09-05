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

import com.qualcomm.hardware.hitechnic.HiTechnicNxtGyroSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This is an example LinearOpMode that shows how to use a legacy (NXT-compatible)
 * HiTechnic gyroscope. It assumes that the gyroscope is configured with a name of "gyro".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Autonomous(name = "Sensor: HT Gyro", group = "Sensor")
@Disabled
public class SensorHTGyro extends LinearOpMode {

    /** In this sample, for illustration purposes we use two interfaces on the one gyro object.
     * That's likely atypical: you'll probably use one or the other in any given situation,
     * depending on what you're trying to do. {@link IntegratingGyroscope} (and it's base interface,
     * {@link Gyroscope}) are common interfaces supported by possibly several different gyro
     * implementations. {@link HiTechnicNxtGyroSensor}, by contrast, provides functionality that
     * is unique to the HiTechnic gyro sensor.
     */
    Gyroscope gyroscope;
    HiTechnicNxtGyroSensor hiTechnicNxtGyroSensor;

    @Override public void runOpMode() throws InterruptedException {

        // Get a reference to the gyroscope from the hardware map
        gyroscope = hardwareMap.get(Gyroscope.class, "gyro");

        // Get a reference to the *implementation* of the gyroscope on the HiTechnic sensor.
        // Usually, you won't need to examine internal implementation details in this way, but
        // we do so to illustrate aspects of what is going on inside the sensor.
        hiTechnicNxtGyroSensor = hardwareMap.get(HiTechnicNxtGyroSensor.class, "gyro");
        // Alternately, we could have cast: hiTechnicNxtGyroSensor = (HiTechnicNxtGyroSensor)gyro;

        // Optionally, calibrate the gyro to establish a good value for its "zero deg/s" bias
        // voltage value. Calibration is not entirely necessary, as the default bias voltage
        // usually does a reasonable job. While calibration is performed, the gyro must remain
        // motionless. Note that for this gyro sensor, calibration data is not persistently
        // written to EEPROM, but rather should be performed each run.
        telemetry.log().add("calibrating...");
        hiTechnicNxtGyroSensor.calibrate(3000, 100);
        telemetry.log().add("...done...waiting for start...");

        // wait for the start button to be pressed.
        waitForStart();
        telemetry.log().clear();

        // loop until the opmode has been asked to stop
        while (opModeIsActive()) {

            double raw = hiTechnicNxtGyroSensor.readRawVoltage();
            double bias = hiTechnicNxtGyroSensor.getBiasVoltage();

            telemetry.addData("rate", "%.4f deg/s",      gyroscope.getAngularVelocity(AngleUnit.DEGREES).zRotationRate);
            telemetry.addData("raw ", "%.4fv",           raw);
            telemetry.addData("bias", "%.4fv",           bias);
            telemetry.addData("volts", "%.4fv",          raw-bias);
            telemetry.addData("deg/s/v", "%.4f deg/s/v", hiTechnicNxtGyroSensor.getDefaultDegreesPerSecondPerVolt());

            telemetry.update();
            idle();
        }
    }
}
