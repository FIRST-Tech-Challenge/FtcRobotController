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

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Test: Color Sensor", group="Test")
public class Test_ColorSensor extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private double redBias = 255.0 / 65535.0;
    private double greenBias = 255.0 / 65535.0;
    private double blueBias = 255.0 / 65535.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        RevColorSensorV3 colorSensor = hardwareMap.get(RevColorSensorV3.class, "clr_sensor");
        FtcDashboard dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("status", "alive");
        dashboard.sendTelemetryPacket(packet);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.b) { // red
                redBias = 255.0 / (double) colorSensor.red();
            }
            if (gamepad1.a) { // green
                greenBias = 255.0 / (double) colorSensor.green();
            }
            if (gamepad1.x) { // blue
                blueBias = 255.0 / (double) colorSensor.blue();
            }
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Unbiased Color Data", "Red: %d, Green: %d, Blue: %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
            telemetry.addData("Biased Color Data", "Red: %f, Green: %f, Blue: %f", getBiased(colorSensor.red(), redBias, "red"), getBiased(colorSensor.green(), greenBias, "green"), getBiased(colorSensor.blue(), blueBias, "blue"));
            telemetry.addData("Sensor Bias", "Red: %f, Green: %f, Blue: %f", redBias, greenBias, blueBias);
            telemetry.addData("Color Sensor", "Light: %f", 1-colorSensor.getLightDetected());
            telemetry.addData("Color Sensor", "Distance: %f inches", colorSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }

    private double getBiased(int value, double bias, String color) {
        double out = (double) value * bias;
        if (out<0 || out>255) {
            telemetry.addData("Bad Bias", "Warning: biased value of color %s has gone out of range. Please adjust bias.", color);
        }
        return Range.clip(out, 0, 255);
    }
}
