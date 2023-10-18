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

package org.firstinspires.ftc.teamcode.Concept;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Concept.DrawRectangleProcessor;
import org.firstinspires.ftc.teamcode.Concept.Draw3RectanglesProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import org.opencv.core.Mat;
import org.opencv.core.Rect;


/*
 * This example drives the test case for FTC OpenCV apps using a custom Vision Portal
 * illustrated by Alan G Smith in his book Learn Java for FTC
 * https://raw.githubusercontent.com/alan412/LearnJavaForFTC/master/LearnJavaForFTC.pdf
 *
 * This OpMode has been modified to align with the idiosyncracies of baqwas
 */

@TeleOp(name="OpenCV: Hello", group="Concept")
@Disabled
public class HelloOpenCV extends LinearOpMode {

    // Declare OpMode members
    //private DrawRectangleProcessor drawRectangleProcessor;
    private Draw3RectanglesProcessor draw3RectangleProcessor;
    private VisionPortal visionPortal;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //drawRectangleProcessor = new DrawRectangleProcessor();
        draw3RectangleProcessor = new Draw3RectanglesProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam1"),
                draw3RectangleProcessor);




        waitForStart();     // Wait for Driver to press PLAY button on Driver Hub
        runtime.reset();    // reset the timer counter


        while (opModeIsActive()) {  // run until the Driver presses the STOP button



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.update();
        }
    }
}
