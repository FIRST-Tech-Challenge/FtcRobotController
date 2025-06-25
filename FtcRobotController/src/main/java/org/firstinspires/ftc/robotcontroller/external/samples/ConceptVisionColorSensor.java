/*
 * Copyright (c) 2024 Phil Malone
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

/*
 * This OpMode illustrates how to use a video source (camera) as a color sensor
 *
 * A "color sensor" will typically determine the color of the object that it is pointed at.
 *
 * This sample performs the same function, except it uses a video camera to inspect an object or scene.
 * The user may choose to inspect all, or just a Region of Interest (ROI), of the active camera view.
 * The user must also provide a list of "acceptable colors" (Swatches) from which the closest matching color will be selected.
 *
 * To perform this function, a VisionPortal runs a PredominantColorProcessor process.
 *   The PredominantColorProcessor process is created first, and then the VisionPortal is built to use this process.
 *   The PredominantColorProcessor analyses the ROI and splits the colored pixels into several color-clusters.
 *   The largest of these clusters is then considered to be the "Predominant Color"
 *   The process then matches the Predominant Color with the closest Swatch and returns that match.
 *
 * To aid the user, a colored rectangle is drawn on the camera preview to show the RegionOfInterest,
 * The Predominant Color is used to paint the rectangle border, so the user can verify that the color is reasonable.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Disabled
@TeleOp(name = "Concept: Vision Color-Sensor", group = "Concept")
public class ConceptVisionColorSensor extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        /* Build a "Color Sensor" vision processor based on the PredominantColorProcessor class.
         *
         * - Focus the color sensor by defining a RegionOfInterest (ROI) which you want to inspect.
         *     This can be the entire frame, or a sub-region defined using:
         *     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
         *     Use one form of the ImageRegion class to define the ROI.
         *         ImageRegion.entireFrame()
         *         ImageRegion.asImageCoordinates(50, 50,  150, 150)  100x100 pixel square near the upper left corner
         *         ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1)  10% width/height square centered on screen
         *
         * - Set the list of "acceptable" color swatches (matches).
         *     Only colors that you assign here will be returned.
         *     If you know the sensor will be pointing to one of a few specific colors, enter them here.
         *     Or, if the sensor may be pointed randomly, provide some additional colors that may match the surrounding.
         *     Possible choices are:
         *         RED, ORANGE, YELLOW, GREEN, CYAN, BLUE, PURPLE, MAGENTA, BLACK, WHITE;
         *
         *     Note that in the example shown below, only some of the available colors are included.
         *     This will force any other colored region into one of these colors.
         *     eg: Green may be reported as YELLOW, as this may be the "closest" match.
         */
        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();

        /*
         * Build a vision portal to run the Color Sensor process.
         *
         *  - Add the colorSensor process created above.
         *  - Set the desired video resolution.
         *      Since a high resolution will not improve this process, choose a lower resolution that is
         *      supported by your camera.  This will improve overall performance and reduce latency.
         *  - Choose your video source.  This may be
         *      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
         *  or
         *      .setCamera(BuiltinCameraDirection.BACK)    ... for a Phone Camera
         */
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(50);  // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
        while (opModeIsActive() || opModeInInit())
        {
            telemetry.addLine("Preview on/off: 3 dots, Camera Stream\n");

            // Request the most recent color analysis.
            // This will return the closest matching colorSwatch and the predominant color in RGB, HSV and YCrCb color spaces.
            // The color space values are returned as three-element int[] arrays as follows:
            //  RGB   Red 0-255, Green 0-255, Blue 0-255
            //  HSV   Hue 0-180, Saturation 0-255, Value 0-255
            //  YCrCb Luminance(Y) 0-255, Cr 0-255 (center 128), Cb 0-255 (center 128)
            //
            // Note: to take actions based on the detected color, simply use the colorSwatch or color space value in a comparison or switch.
            //  eg:
            //      if (result.closestSwatch == PredominantColorProcessor.Swatch.RED) {... some code  ...}
            //  or:
            //      if (result.RGB[0] > 128) {... some code  ...}

            PredominantColorProcessor.Result result = colorSensor.getAnalysis();

            // Display the Color Sensor result.
            telemetry.addData("Best Match", result.closestSwatch);
            telemetry.addLine(String.format("RGB   (%3d, %3d, %3d)", result.RGB[0], result.RGB[1], result.RGB[2]));
            telemetry.addLine(String.format("HSV   (%3d, %3d, %3d)", result.HSV[0], result.HSV[1], result.HSV[2]));
            telemetry.addLine(String.format("YCrCb (%3d, %3d, %3d)", result.YCrCb[0], result.YCrCb[1], result.YCrCb[2]));
            telemetry.update();

            sleep(20);
        }
    }
}
