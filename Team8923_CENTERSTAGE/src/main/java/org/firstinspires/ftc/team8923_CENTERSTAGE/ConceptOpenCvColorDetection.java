/*
 * Copyright (c) 2019 OpenFTC Team
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
 *
 * Modified by Truong Nguyen
 * 2023/11/03
 * Example program to detect blue or red objects
 */

package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * This program demonstrates the use of EasyOpenCV to detect blue or red objects
 * It uses the USB webcam named "RobotCamera" in the robot configuration,
 *    that is connected to the REV control hub
 * It instantiates the OpenCvColorDetection class
 *
 * Use the Logitech F310 game pad, User 1 (press Start-A)
 *   Press the blue X button to detect blue objects
 *   Press the red B button to detect red objects
 * Run scrcpy on a computer to see the image pipeline on the control hub
 */

@TeleOp(name = "OpenCV Color Detection Example", group = "Concept")
// @Disabled
public class ConceptOpenCvColorDetection extends LinearOpMode {
    // create the openCV detection object
    //   pass (opmode: this) to the object so it could access the camera hardware
    OpenCvColorDetection myColorDetection = new OpenCvColorDetection(this);

    @Override
    public void runOpMode()
    {
        // initialize the webcam and openCV pipeline
        myColorDetection.init();

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            // print detection status and coordinates of largest object
            telemetry.addLine("Detection")
                    .addData(" ", myColorDetection.targetDetected)
                    .addData("x", myColorDetection.targetPoint.x)
                    .addData("y", myColorDetection.targetPoint.y);

            telemetry.addData("Frame Count", myColorDetection.robotCamera.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", myColorDetection.robotCamera.getFps()));
            telemetry.addData("Total frame time ms", myColorDetection.robotCamera.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", myColorDetection.robotCamera.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", myColorDetection.robotCamera.getOverheadTimeMs());
            telemetry.update();

            // Change detection color with game pad buttons
            if(gamepad1.x){
                myColorDetection.setDetectColor(OpenCvColorDetection.detectColorType.BLUE);
            }

            if(gamepad1.b){
                myColorDetection.setDetectColor(OpenCvColorDetection.detectColorType.RED);
            }

            // start and stop pipeline
            if(gamepad1.y){
                myColorDetection.startStreaming();
            }

            if(gamepad1.a){
                myColorDetection.stopStreaming();
            }
        }
        myColorDetection.robotCamera.stopStreaming();
        myColorDetection.robotCamera.closeCameraDevice();
    }
}
