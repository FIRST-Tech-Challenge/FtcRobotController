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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/*
 * This class demonstrates the use of EasyOpenCV to detect color objects in HSV space
 * It uses the USB webcam named "RobotCamera" in the robot configuration, connected to the REV control hub
 */

public class OpenCvColorDetection {
    OpenCvCamera robotCamera;
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    enum detectColorType{
        BLUE,
        RED
    }

    public enum Position {
        ONE,
        TWO,
        THREE,
        FOUR
    }

    detectColorType myColor;

    // coordinates of largest detected image
    Point targetPoint = new Point(0, 0);
    boolean targetDetected;

    // Define a constructor that allows the OpMode to pass a reference to itself
    //   in order to allow this class to access the camera hardware
    public OpenCvColorDetection(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    // set detection color
    public void setDetectColor(detectColorType color) {
        myColor = color;
    }

    // initialize the camera and openCV pipeline
    public void init() {

        // cameraMonitorViewId allows us to see the image pipeline using scrcpy
        //   for easy debugging
        //   You can disable it after testing completes
        int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
        robotCamera = OpenCvCameraFactory.getInstance().createWebcam(myOpMode.hardwareMap.get(WebcamName.class, "RobotCamera"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        // robotCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"));
        // OR... use internal phone camera
        // phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

       setDetectColor(detectColorType.BLUE);
        robotCamera.setPipeline(new ColorDetectPipeline());

        robotCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened()
            {
                startStreaming();
            }

            public void onError(int errorCode) {
                stopStreaming();
            }
        });
    }

    public void stopStreaming() {
        robotCamera.stopStreaming();
    }

    public void startStreaming() { // resolution of camera stream
        robotCamera.startStreaming(ConstantsOpenCV.CAMERA_IMAGE_WIDTH, ConstantsOpenCV.CAMERA_IMAGE_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

    /* openCV image processing pipeline
     *   detects largest blue or red colored object
     */
    class ColorDetectPipeline extends OpenCvPipeline {
        boolean viewportPaused = false;

        // matrices in the processing pipeline
        Mat blurredMat = new Mat();
        Mat hsvMat = new Mat();
        Mat filteredMat = new Mat();
        Mat contourMask = new Mat();
        Mat outputMat = new Mat();

        @Override
        public Mat processFrame(Mat inputMat) {
            // blur the image to reduce the impact of noisy pixels
            //   each pixel is "averaged" with its neighboring pixels
            Imgproc.GaussianBlur(inputMat, blurredMat, ConstantsOpenCV.BLUR_SIZE, 0);

            // convert image to HSV color space, which is better for detecting red and blue colors
            Imgproc.cvtColor(blurredMat, hsvMat, Imgproc.COLOR_RGB2HSV);

            // filter out the range of blue or red colors defined in
            //   Constants.BLUE_COLOR_DETECT_MIN_HSV and Constants.BLUE_COLOR_DETECT_MAX_HSV
            //   or Constants.RED_COLOR_DETECT_MIN_HSV and Constants.RED_COLOR_DETECT_MAX_HSV
            if (myColor == detectColorType.BLUE) {
                Core.inRange(hsvMat, ConstantsOpenCV.BLUE_COLOR_DETECT_MIN_HSV, ConstantsOpenCV.BLUE_COLOR_DETECT_MAX_HSV, filteredMat);
            } else {
                Core.inRange(hsvMat, ConstantsOpenCV.RED_COLOR_DETECT_MIN_HSV, ConstantsOpenCV.RED_COLOR_DETECT_MAX_HSV, filteredMat);
            }

            // create a list of contours surrounding groups of contiguous pixels that were filtered
            List<MatOfPoint> contoursList = new ArrayList<>();
            Imgproc.findContours(filteredMat, contoursList, contourMask, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            // copy original image to output image for drawing overlay on
            inputMat.copyTo(outputMat);

            //   iterate through list of contours, find max area contour
            int maxAreaContourIndex = -1;
            targetPoint.x = -1;
            targetPoint.y = -1;
            targetDetected = false;
            if (contoursList.size() > 1) {
                double maxArea = 0.0;
                maxAreaContourIndex = 0;
                for (int i = 0; i < contoursList.size(); i++) {
                    if (Imgproc.contourArea(contoursList.get(i)) > maxArea) {
                        maxArea = Imgproc.contourArea(contoursList.get(i));
                        maxAreaContourIndex = i;
                    }
                }
                targetDetected = true;
                // Draw the max area contour at index maxAreaContourIndex for debugging in scrcpy
                Imgproc.drawContours(outputMat, contoursList, maxAreaContourIndex, ConstantsOpenCV.borderColor, 2, -1);

                // draw rectangular bounding box around the max area contour
                //   and draw circle at the center of rectangular bounding box
                Rect boundingRect = Imgproc.boundingRect(contoursList.get(maxAreaContourIndex));
                double boundHeightX = boundingRect.x + boundingRect.width;
                double boundHeightY = boundingRect.y + boundingRect.height;
                Imgproc.rectangle(outputMat, new Point(boundingRect.x, boundingRect.y), new Point(boundHeightX, boundHeightY), ConstantsOpenCV.borderColor, 3, Imgproc.LINE_8, 0);
                targetPoint.x = (int) boundingRect.width / 2.0 + boundingRect.x;
                targetPoint.y = (int) boundingRect.height / 2.0 + boundingRect.y;
                Imgproc.circle(outputMat, targetPoint, 10, ConstantsOpenCV.borderColor, Imgproc.LINE_4, -1);
            }

            // See this image on the computer using scrcpy
            return outputMat;
        }

        //@Override
        /*public void onViewportTapped() {
            viewportPaused = !viewportPaused;

            if(viewportPaused) {
                robotCamera.pauseViewport();
            }
            else {
                robotCamera.resumeViewport();
            }
        }*/
    }

    public Position detectColor() {
        Position position = Position.FOUR;
        if (targetPoint.x < 213.3) {
            position = Position.ONE;
        } else if ((targetPoint.x > 213.3) && (targetPoint.x < 426.6)) {
            position = Position.TWO;
        } else if ((targetPoint.x > 426.6) && (targetPoint.x < 640)) {
            position = Position.THREE;
        } else {
            position = Position.FOUR;
        }
        return position;
    }
}
