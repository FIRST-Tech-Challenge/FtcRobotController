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

import static java.lang.Double.NaN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;


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

@TeleOp(name="Calibrate Color Sensor", group="Calibration")
public class Calibrate_ColorSensor extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private double redBias = 255.0 / 65535.0;
    private double greenBias = 255.0 / 65535.0;
    private double blueBias = 255.0 / 65535.0;
    private RevColorSensorV3 colorSensor = null;

    enum BlockColor {
        UNKNOWN,
        RED,
        YELLOW,
        BLUE
    }

    enum RGBColor {
        RED,
        GREEN,
        BLUE
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "clr_sensor");
        FtcDashboard dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("status", "alive");
        dashboard.sendTelemetryPacket(packet);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        long startingTime = 0;
        BlockColor tuningWhat = BlockColor.UNKNOWN;
        List<RGBColorValue[]> history = new ArrayList<>();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.b) { // red
                startingTime = runtime.nanoseconds();
                tuningWhat = BlockColor.RED;
            }
            if (gamepad1.y) { // yellow
                startingTime = runtime.nanoseconds();
                tuningWhat = BlockColor.YELLOW;
            }
            if (gamepad1.x) { // blue
                startingTime = runtime.nanoseconds();
                tuningWhat = BlockColor.BLUE;
            }
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Unbiased Color Data", "Red: %d, Green: %d, Blue: %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
            telemetry.addData("Biased Color Data", "Red: %f, Green: %f, Blue: %f", getBiased(colorSensor.red(), redBias, "red"), getBiased(colorSensor.green(), greenBias, "green"), getBiased(colorSensor.blue(), blueBias, "blue"));
            telemetry.addData("Sensor Bias", "Red: %f, Green: %f, Blue: %f", redBias, greenBias, blueBias);
            telemetry.addData("Color Sensor", "Light: %f", 1-colorSensor.getLightDetected());
            telemetry.addData("Color Sensor", "Distance: %f inches", colorSensor.getDistance(DistanceUnit.INCH));
            telemetry.addLine();
            telemetry.addData("Read Block Color", getBlockColor().toString());

            if (tuningWhat != BlockColor.UNKNOWN) {
                history.add(getColorOrder());
            }
            if (tuningWhat != BlockColor.UNKNOWN && runtime.nanoseconds()-startingTime >= (1000 * 1000 * 1000)) { // 1 second
                tune(tuningWhat, history);
                tuningWhat = BlockColor.UNKNOWN;
                startingTime = 0;
                history.clear();
            }

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

    private RGBColorValue[] getOrderOfRGB(double red, double green, double blue) {
        // Create an array of RGBColor and their corresponding values
        RGBColorValue[] colorValues = {
                new RGBColorValue(RGBColor.RED, red),
                new RGBColorValue(RGBColor.GREEN, green),
                new RGBColorValue(RGBColor.BLUE, blue)
        };

        // Sort the array by their values in ascending order
        Arrays.sort(colorValues, Comparator.comparingDouble(c -> -c.value));

        return colorValues;
    }

    // Helper class to store RGBColor and its value
    private static class RGBColorValue {
        RGBColor color;
        double value;

        RGBColorValue(RGBColor color, double value) {
            this.color = color;
            this.value = value;
        }
    }

    private double getColorsValue(RGBColor color, RGBColorValue[] values) {
        for (RGBColorValue value : values) {
            if (value.color == color) {
                return value.value;
            }
        }
        return NaN;
    }

    private void tune(BlockColor tuningWhat, List<RGBColorValue[]> history) {
        double maxRed = 0.0, maxGreen = 0.0, maxBlue = 0.0;
        for (RGBColorValue[] colorValues : history) {

            switch (tuningWhat) {
                case RED:
                    if (redBias*getColorsValue(RGBColor.RED, colorValues)<230) {
                        redBias = 235.0 / getColorsValue(RGBColor.RED, colorValues);
                    }
                case YELLOW:
                    if (greenBias*getColorsValue(RGBColor.GREEN, colorValues)<230) {
                        greenBias = 235.0 / getColorsValue(RGBColor.GREEN, colorValues);
                    }
                case BLUE:
                    if (blueBias*getColorsValue(RGBColor.BLUE, colorValues)<230) {
                        blueBias = 235.0 / getColorsValue(RGBColor.BLUE, colorValues);
                    }
            }
        }
    }

    private double getBias(RGBColor color) {
        switch (color) {
            case RED:
                return redBias;
            case BLUE:
                return blueBias;
            case GREEN:
                return greenBias;
            default:
                return NaN;
        }
    }

    private RGBColorValue[] fromBiased(RGBColorValue[] colors) {
        RGBColorValue[] out = {null, null, null};
        for (int i = 0; i < 3; i++) {
            out[i] = colors[i];
            out[i].value = out[i].value / getBias(out[i].color);
        }
        return out;
    }

    private RGBColorValue[] getColorOrder() {
        return getOrderOfRGB(getBiased(colorSensor.red(), redBias, "red"), getBiased(colorSensor.green(), greenBias, "green"), getBiased(colorSensor.blue(), blueBias, "blue"));
    }

    private BlockColor getBlockColor() {
        return getBlockColor(getColorOrder());
    }

    private BlockColor getBlockColor(RGBColorValue[] colorOrder) {
        switch (colorOrder[0].color) {
            case RED:
                return BlockColor.RED;
            case BLUE:
                return BlockColor.BLUE;
            case GREEN:
                return BlockColor.YELLOW;
            default:
                throw new RuntimeException("Got no value for colorOrder[0].color, something is VERY wrong!");
        }
    }
}
