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

package org.firstinspires.ftc.teamcode.Testing.Teleop_test;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@TeleOp(name = "Sensor: REVColorDistanceTouch", group = "Sensor")
//@Disabled                            // Comment this out to add to the opmode list
public class ColorSensorDistanceTouch extends LinearOpMode {

    /**
     * Note that the REV Robotics Color-Distance incorporates two sensors into one device.
     * It has a light/distance (range) sensor.  It also has an RGB color sensor.
     * The light/distance sensor saturates at around 2" (5cm).  This means that targets that are 2"
     * or closer will display the same value for distance/light detected.
     *
     * Although you configure a single REV Robotics Color-Distance sensor in your configuration file,
     * you can treat the sensor as two separate sensors that share the same name in your op mode.
     *
     * In this example, we represent the detected color by a hue, saturation, and value color
     * model (see https://en.wikipedia.org/wiki/HSL_and_HSV).  We change the background
     * color of the screen to match the detected color.
     *
     * In this example, we  also use the distance sensor to display the distance
     * to the target object.  Note that the distance sensor saturates at around 2" (5 cm).
     *
     */
    ColorSensor sensorColor;
    ColorSensor sensorColor2;
    DistanceSensor sensorDistance;
    DistanceSensor sensorDistance2;
    TouchSensor    touchSensor;
    TouchSensor touchSensor2;
    @Override
    public void runOpMode() {
        double delay = 2.5;
        String foundColor = "null";
        ElapsedTime runtime = new ElapsedTime();

        double lastLeftTime = -(delay) - 0.01; //so that it wouldn't light up at the start
        double lastRightTime = -(delay) - 0.01; //so that it wouldn't light up at the start
        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorColor2 = hardwareMap.get(ColorSensor.class, "sensor_color_distance_2");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        sensorDistance2 = hardwareMap.get(DistanceSensor.class, "sensor_color_distance_2");
        // get a reference to our digitalTouch object.
//        touchSensor = hardwareMap.get(RevTouchSensor.class, "sensor_digital");
//        touchSensor2 = hardwareMap.get(RevTouchSensor.class, "sensor_digital_2");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};
        float hsv2Values[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        final float values2[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/accentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // wait for the start button to be pressed.
        waitForStart();

        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)

            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            //touch sensor stuff below
            // if the digital channel returns true it's HIGH and the button is unpressed.
//            if ( touchSensor.isPressed() ) {
//                telemetry.addData("Left Sensor", "Is Currently Pressed");
//                lastLeftTime = runtime.seconds();
//            } else
//                telemetry.addData("Left Sensor", "Is Not Currently Pressed");
//            if ( touchSensor2.isPressed() ) {
//                telemetry.addData("Right Sensor", "Is Currently Pressed");
//                lastRightTime = runtime.seconds();
//            } else
//                telemetry.addData("Right Sensor", "Is Not Currently Pressed");
//            // send the info back to driver station using telemetry function.
//            if(runtime.seconds() - lastLeftTime <= delay){
//                telemetry.addData("Left S ide", "Hit");
//            }
//            else{
//                telemetry.addData("Left Side", "Not Hit");
//
//            }
//            if(runtime.seconds() - lastRightTime <= delay){
//                telemetry.addData("Right Side", "Hit");
//            }
//            else{
//                telemetry.addData("Right Side", "Not Hit");
//            }

            telemetry.addData("Left Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Right Distance (cm)",
                   String.format(Locale.US, "%.02f", sensorDistance2.getDistance(DistanceUnit.CM)));
//            telemetry.addLine("Alpha");
//            telemetry.addLine("Red  ");
//            telemetry.addLine("Green");
//            telemetry.addLine("Blue ");
//            telemetry.addData("Hue", hsvValues[0]);
//            telemetry.addData("Saturation", hsvValues[1]);
//            telemetry.addData("Value", hsvValues[2]);
//            telemetry.addData("Red", sensorColor.red());
//            telemetry.addData("Blue", sensorColor.blue());
//            telemetry.addData("Green", sensorColor.green());
            // convert the RGB values to HSV values.
// multiply by the SCALE_FACTOR.
// then cast it back to int (SCALE_FACTOR is a double)

            Color.RGBToHSV((int) (sensorColor2.red() * SCALE_FACTOR),
                    (int) (sensorColor2.green() * SCALE_FACTOR),
                    (int) (sensorColor2.blue() * SCALE_FACTOR),
                    hsv2Values);

            String leftdetectedColor = "Unknown";
            String rightdetectedColor = "Unknown";
            double hue = hsvValues[0];
            double hue2 = hsv2Values[0];
            if (hue >= 0 && hue < 60 || hue > 360) {
                leftdetectedColor = "Red";
            } else if (hue >= 60 && hue < 120) {
                leftdetectedColor = "Yellow";
            } else if (hue >= 120 && hue < 150) {
                leftdetectedColor = "Green";
            } else if (hue >= 210 && hue < 300) {
                leftdetectedColor = "Blue";
            } else if (hue >= 180 && hue < 210) {
                leftdetectedColor = "Purple";
            } else if(hue > 150 && hue < 180){
                leftdetectedColor = "White";
            }

            if (hue2 >= 0 && hue2 < 60 || hue2 > 360) {
                rightdetectedColor = "Red";
            } else if (hue2 >= 60 && hue2 < 120) {
                rightdetectedColor = "Yellow";
            } else if (hue2 >= 120 && hue2 < 150) {
                rightdetectedColor = "Green";
            } else if (hue2 >= 210 && hue2 < 300) {
                rightdetectedColor = "Blue";
            } else if (hue2 >= 180 && hue2 < 210) {
                rightdetectedColor = "Purple";
            } else if(hue2 > 150 && hue2 < 180){
                rightdetectedColor = "White";
            }

            telemetry.addData("Left Color", leftdetectedColor);
            telemetry.addData("Right Color", rightdetectedColor);
            telemetry.update();


//            double red = sensorColor.red();
//            double blue = sensorColor.blue();
//            double green = sensorColor.green();
//            if(red > blue * 2 && red > green * 2){
//                telemetry.addData("Color: ", "Red");
//            }
//            else if(blue > red * 2 && blue > green * 2){
//                telemetry.addData("Color: ", "Blue");
//            }
//            else if(green > red * 2 && green > blue * 2){
//                telemetry.addData("Color: ", "Green");
//            }
//            else if(green * 2 < red && green * 2 < blue){
//                telemetry.addData("Color: ", "Purple");
//            }
//            else if(blue * 2 < red && blue * 2 < green){
//                telemetry.addData("Color: ", "Yellow");
//            }
//            else {
//                telemetry.addData("Color: ", "No Other");
//            }
//            telemetry.addData("Red", red);
//            telemetry.addData("Green", green);
//            telemetry.addData("Blue", blue);


//            if (sensorColor.red()>sensorColor.green()&& sensorColor.red()>sensorColor.blue()) {
//                telemetry.addData("Color Detected", "Red");
//            }
//            if (sensorColor.red() < sensorColor.green() && sensorColor.blue() < sensorColor.green()) {
//                telemetry.addData("Color Detected", "Green");
//            }
//
//            if (sensorColor.red() <  sensorColor.blue() && sensorColor.green() <  sensorColor.blue()) {
//                telemetry.addData("Color Detected", "Blue");
//            }
//
//            if (sensorColor.red() <  sensorColor.blue() && sensorColor.green() <  sensorColor.blue()) {
//                telemetry.addData("Color Detected", "Blue");
//            }
//
//
//            if (sensorColor.red() <  sensorColor.blue() && sensorColor.green() <  sensorColor.blue()) {
//                telemetry.addData("Color Detected", "Blue");
//            }


            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();
        }

        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }
}
