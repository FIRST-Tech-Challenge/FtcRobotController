/*
 * Copyright (c) 2020 OpenFTC Team
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "TestComputerVisionShippingElement")
public class EasyOpenCVIdentifyShippingElement extends LinearOpMode {
    OpenCvWebcam webcam;
    SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline(telemetry);
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        // webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Can't open camera");
                telemetry.update();
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();
        telemetry.addData("Position", "HELLO");
        telemetry.update();

        while (opModeIsActive()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Analysis2", pipeline.getAnalysis2());
            telemetry.addData("Analysis3", pipeline.getAnalysis3());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
        Telemetry telemetry;

        public SkystoneDeterminationPipeline(Telemetry telemetry) {
            this.telemetry = telemetry;
        }

        /*
         * An enum to define the skystone position
         */
        public enum FreightPosition {
            LEFT,
            MIDDLE,
            RIGHT
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar RED = new Scalar(255, 0, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOP_LEFT_ANCHOR_POINT = new Point(10, 200);
        static final Point REGION2_TOP_LEFT_ANCHOR_POINT = new Point(312, 210);
        static final Point REGION3_TOP_LEFT_ANCHOR_POINT = new Point(607, 210);


        static final int REGION_WIDTH = 30;
        static final int REGION_HEIGHT = 42;

        final int FREIGHT_PRESENT_THRESHOLD = 110;

        Point region1_pointA = new Point(
                REGION1_TOP_LEFT_ANCHOR_POINT.x,
                REGION1_TOP_LEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOP_LEFT_ANCHOR_POINT.x,
                REGION2_TOP_LEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Point region3_pointA = new Point(
                REGION3_TOP_LEFT_ANCHOR_POINT.x,
                REGION3_TOP_LEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_A;
        Mat region2_A;
        Mat region3_A;
        Mat LAB = new Mat();

        Mat A = new Mat();
        int avg1 = 0;
        int avg2 = 0;
        int avg3 = 0;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile FreightPosition position = FreightPosition.LEFT;

        /*
         * This function takes the RGB frame, converts to LAB,
         * and extracts the A channel to the 'A' variable*/

        void inputToLAB_A(Mat input) {

            Imgproc.cvtColor(input, LAB, Imgproc.COLOR_RGB2Lab);
            Core.extractChannel(LAB, A, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToLAB_A(firstFrame);

            region1_A = A.submat(new Rect(region1_pointA, region1_pointB));
            region2_A = A.submat(new Rect(region2_pointA, region2_pointB));
            region3_A = A.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToLAB_A(input);
            region1_A = A.submat(new Rect(region1_pointA, region1_pointB));
            region2_A = A.submat(new Rect(region2_pointA, region2_pointB));
            region3_A = A.submat(new Rect(region3_pointA, region3_pointB));

            avg1 = (int) Core.mean(region1_A).val[0];
            avg2 = (int) Core.mean(region2_A).val[0];
            avg3 = (int) Core.mean(region3_A).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            if (avg1 < FREIGHT_PRESENT_THRESHOLD) {
                position = FreightPosition.LEFT;
            } else if (avg2 < FREIGHT_PRESENT_THRESHOLD) {
                position = FreightPosition.MIDDLE;
            } else {
                position = FreightPosition.RIGHT;
            }
            telemetry.addData("Analysis", avg1);
            telemetry.addData("Analysis2", avg2);
            telemetry.addData("Analysis3", avg3);
            telemetry.addData("Position", position);
            telemetry.update();


            return input;
        }

        public int getAnalysis() {
            return avg1;
        }

        public int getAnalysis2() {
            return avg2;
        }

        public int getAnalysis3() {
            return avg3;
        }


    }


}