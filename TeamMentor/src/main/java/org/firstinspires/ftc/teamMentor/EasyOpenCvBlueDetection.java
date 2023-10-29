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
 */

package org.firstinspires.ftc.teamMentor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/*
 * This program demonstrates the use of EasyOpenCV to detect blue objects
 * It uses the USB webcam named "RobotCamera" in the robot configuration, connected to the REV control hub
 */

@TeleOp(name = "Blue Detection Example", group = "Concept")
// @Disabled
public class EasyOpenCvBlueDetection extends LinearOpMode {
    OpenCvCamera robotCamera;

    @Override
    public void runOpMode()
    {

        // cameraMonitorViewId allows us to see the image pipeline using scrcpy
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robotCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        // robotCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"));
        // phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        robotCamera.setPipeline(new SamplePipeline());

        robotCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                robotCamera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Frame Count", robotCamera.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", robotCamera.getFps()));
            telemetry.addData("Total frame time ms", robotCamera.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", robotCamera.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", robotCamera.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", robotCamera.getCurrentPipelineMaxFps());
            telemetry.update();
            if(gamepad1.a)
            {

                robotCamera.stopStreaming();
                //robotCamera.closeCameraDevice();
            }
            sleep(100);
        }
    }

    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused = false;
        Mat input = new Mat();
        Mat threshold = new Mat();
        Mat contourMask = new Mat();
        Mat output = new Mat();

        @Override
        public Mat processFrame(Mat input)
        {
            // blur the image to reduce the impact of noisy pixels
            //   each pixel is "averaged" with its neighboring pixels
            Imgproc.GaussianBlur(input, this.input, Constants.BLUR_SIZE, 0);

            // convert image to HSV color space, which is better for detecting red and blue colors
            Imgproc.cvtColor(this.input, this.input, Imgproc.COLOR_RGB2HSV);

            // filter out the range of blue colors defined in Constants.BLUE_COLOR_DETECT_MIN_HSV
            //   and Constants.BLUE_COLOR_DETECT_MAX_HSV
            //   Those filtered pixels show up in "threshold" as 255.  Other pixels "threshold" are 0.
            Core.inRange(this.input, Constants.BLUE_COLOR_DETECT_MIN_HSV, Constants.BLUE_COLOR_DETECT_MAX_HSV, this.input);
            Imgproc.threshold(this.input, threshold, 1, 255,Imgproc.THRESH_BINARY);

            // create a list of contours surrounding groups of contiguous pixels that were filtered
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(threshold, contours, contourMask, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            //   iterate through list of contours, find max area contour
            int maxAreaContourIndex = -1;
            if (contours.size() > 1) {
                double maxArea = 0;
                maxAreaContourIndex = -1;
                for (int i = 0; i < contours.size(); i++) {
                    if (Imgproc.contourArea(contours.get(i)) > maxArea) {
                        maxArea = Imgproc.contourArea(contours.get(i));
                        maxAreaContourIndex = i;
                    }
                }
                // Draw the max area contour at index maxAreaContourIndex for debugging in scrcpy
                Imgproc.drawContours(input, contours, maxAreaContourIndex, Constants.borderColors, 2, -1);

                // draw rectangular bounding box around the max area contour
                //   and draw circle at the center of rectangular bounding box
                Rect boundingRect = Imgproc.boundingRect(contours.get(maxAreaContourIndex));
                double boundHeightX = boundingRect.x + boundingRect.width;
                double boundHeightY = boundingRect.y + boundingRect.height;
                Point circlePoint = new Point(((int)boundingRect.width/2.0 + boundingRect.x), ((int)boundingRect.height/2.0 + boundingRect.y));
                Imgproc.rectangle(input, new Point(boundingRect.x, boundingRect.y), new Point(boundHeightX, boundHeightY), Constants.borderColors, 1, Imgproc.LINE_8, 0);
                Imgproc.circle(input, circlePoint , 10, Constants.borderColors , Imgproc.LINE_8, -1);
            }

            return input;
        }

        @Override
        public void onViewportTapped()
        {
            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                robotCamera.pauseViewport();
            }
            else
            {
                robotCamera.resumeViewport();
            }
        }
    }
}
