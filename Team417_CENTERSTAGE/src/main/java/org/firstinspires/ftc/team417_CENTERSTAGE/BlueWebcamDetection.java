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

package org.firstinspires.ftc.team417_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/*
 * This version of the internal camera example uses EasyOpenCV's interface to the
 * original Android camera API
 * It uses OpenCV for simple and fun edge detection
 * Run this program on an Android phone.  It uses the phone's back camera, and displays
 * the edge detected image on the phone screen.
 */
@Disabled
@TeleOp(name = "Concept: OpenCV Blue Detection", group = "Concept")
// @Disabled
public class BlueWebcamDetection extends LinearOpMode {

    OpenCvCamera robotCamera;

    @Override
    public void runOpMode() {
        robotCamera = initializeCamera();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Frame Count", robotCamera.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", robotCamera.getFps()));
            telemetry.addData("Total frame time ms", robotCamera.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", robotCamera.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", robotCamera.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", robotCamera.getCurrentPipelineMaxFps());
            telemetry.update();
            if (gamepad1.a) {

                robotCamera.stopStreaming();
                //robotCamera.closeCameraDevice();
            }
            sleep(100);
        }
    }

    public OpenCvCamera initializeCamera() {
        OpenCvCamera camera;

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"));

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        camera.setPipeline(new SamplePipeline());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        return camera;
    }

    public static final Scalar LOWER_BLUE = new Scalar(100, 50, 50);
    public static final Scalar UPPER_BLUE = new Scalar(130, 255, 255);

    class SamplePipeline extends OpenCvPipeline {
        boolean viewportPaused = false;
        Mat hsv = new Mat();
        Mat output = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            // convert image to grayscale
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            // blur the image to reduce the impact of noisy pixels
            Imgproc.GaussianBlur(hsv, hsv, new Size(7, 7), 0);
            Core.inRange(hsv, LOWER_BLUE, UPPER_BLUE, hsv);
            Imgproc.threshold(hsv, output, 1, 255, Imgproc.THRESH_BINARY);
            // Resize the binary mask
            Mat resizedMask = new Mat();
            Imgproc.resize(output, resizedMask, new Size(0, 0), 0.7, 0.7, Imgproc.INTER_AREA);

            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(resizedMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            if (contours.size() > 0) {
                // Find the largest contour
                double maxArea = -1;
                int maxAreaIndex = -1;
                for (int i = 0; i < contours.size(); i++) {
                    double area = Imgproc.contourArea(contours.get(i));
                    if (area > maxArea) {
                        maxArea = area;
                        maxAreaIndex = i;
                    }
                }

                MatOfPoint largestContour = contours.get(maxAreaIndex);
                Rect boundingRect = Imgproc.boundingRect(largestContour);
                // Draw a rectangle around the largest contour on the frame
                Imgproc.rectangle(input, boundingRect.tl(), boundingRect.br(), new Scalar(0, 255, 0), 2);

                // Calculate the center of the bounding rectangle
                Point center = new Point(boundingRect.x + (boundingRect.width * 0.5), boundingRect.y + (boundingRect.height * 0.5));

                // Draw the largest contour
                Imgproc.drawContours(input, contours, maxAreaIndex, new Scalar(0, 255, 0));

                // Draw a circle at the center
                Imgproc.circle(input, center, 2, new Scalar(0, 255, 0), 2);
            }
            return input;
        }

        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;

            if (viewportPaused) {
                robotCamera.pauseViewport();
            } else {
                robotCamera.resumeViewport();
            }
        }
    }
}
