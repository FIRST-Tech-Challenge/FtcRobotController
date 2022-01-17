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

import com.qualcomm.robotcore.hardware.HardwareMap;

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


public class FreightFrenzyComputerVisionFrontCamera {
    OpenCvWebcam webcam;
    public DuckPipeline pipeline;
    String color;

    public FreightFrenzyComputerVisionFrontCamera(HardwareMap hardwareMap, Telemetry telemetry) {
        this.color = color;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        pipeline = new DuckPipeline(telemetry);
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
    }


    public static class DuckPipeline extends OpenCvPipeline {
        Telemetry telemetry;

        public DuckPipeline(Telemetry telemetry) {
            this.telemetry = telemetry;
        }

        /*
         * An enum to define the skystone position
         */
        public enum DuckPosition {
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

        static final int FREIGHT_REGION_DISTANCES_FROM_TOP = 210;
        static final int HUB_REGION_DISTANCE_FROM_TOP = 73;
        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOP_LEFT_ANCHOR_POINT = new Point(177, FREIGHT_REGION_DISTANCES_FROM_TOP);
        static final Point REGION2_TOP_LEFT_ANCHOR_POINT = new Point(440, FREIGHT_REGION_DISTANCES_FROM_TOP);
        static final Point REGION3_TOP_LEFT_ANCHOR_POINT = new Point(607, FREIGHT_REGION_DISTANCES_FROM_TOP);

        static final Point REGION1_HUB_TOP_LEFT_ANCHOR_POINT = new Point(0, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point REGION2_HUB_TOP_LEFT_ANCHOR_POINT = new Point(128, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point REGION3_HUB_TOP_LEFT_ANCHOR_POINT = new Point(256, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point REGION4_HUB_TOP_LEFT_ANCHOR_POINT = new Point(384, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point REGION5_HUB_TOP_LEFT_ANCHOR_POINT = new Point(512, HUB_REGION_DISTANCE_FROM_TOP);


        static final int REGION_WIDTH = 30;
        static final int REGION_HEIGHT = 42;

        static final int HUB_REGION_HEIGHT = 50;
        static final int HUB_REGION_WIDTH = 127;

        final int FREIGHT_PRESENT_THRESHOLD = 110;

        final int HUB_PRESENT_THRESHOLD_RED = 140;
        final int HUB_PRESENT_THRESHOLD_BLUE = 129;

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


        //        Hub regions
        Point region1_hub_pointA = new Point(
                REGION1_HUB_TOP_LEFT_ANCHOR_POINT.x,
                REGION1_HUB_TOP_LEFT_ANCHOR_POINT.y);
        Point region1_hub_pointB = new Point(
                REGION1_HUB_TOP_LEFT_ANCHOR_POINT.x + HUB_REGION_WIDTH,
                REGION1_HUB_TOP_LEFT_ANCHOR_POINT.y + HUB_REGION_HEIGHT);

        Point region2_hub_pointA = new Point(
                REGION2_HUB_TOP_LEFT_ANCHOR_POINT.x,
                REGION2_HUB_TOP_LEFT_ANCHOR_POINT.y);
        Point region2_hub_pointB = new Point(
                REGION2_HUB_TOP_LEFT_ANCHOR_POINT.x + HUB_REGION_WIDTH,
                REGION2_HUB_TOP_LEFT_ANCHOR_POINT.y + HUB_REGION_HEIGHT);

        Point region3_hub_pointA = new Point(
                REGION3_HUB_TOP_LEFT_ANCHOR_POINT.x,
                REGION3_HUB_TOP_LEFT_ANCHOR_POINT.y);
        Point region3_hub_pointB = new Point(
                REGION3_HUB_TOP_LEFT_ANCHOR_POINT.x + HUB_REGION_WIDTH,
                REGION3_HUB_TOP_LEFT_ANCHOR_POINT.y + HUB_REGION_HEIGHT);


/*        Dear future Oliver
  * Please not that this is where you left off when adding 2 additional regions
  * to the hub detection cv.
  * There is much left to be done but it was not prioritized.
  * Naming conventions were changed from left, center, right to 1, 2, 3, 4, 5
  *
  */

        /*
         * Working variables
         */
        Mat region1_A;
        Mat region2_A;
        Mat region3_A;
        Mat duckMat = new Mat();

        Mat A = new Mat();

        int avg1 = 0;
        int avg2 = 0;
        int avg3 = 0;


        Mat hub_region_left_A;
        Mat hub_region_center_A;
        Mat hub_region_right_A;

        int hub_avg_left_A = 0;
        int hub_avg_center_A = 0;
        int hub_avg_right_A = 0;

        Mat hub_region_left_B;
        Mat hub_region_center_B;
        Mat hub_region_right_B;

        int hub_avg_left_B = 0;
        int hub_avg_center_B = 0;
        int hub_avg_right_B = 0;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile DuckPosition position = DuckPosition.MIDDLE;



        /*
         * This function takes the RGB frame, converts to LAB,
         * and extracts the A channel to the 'A' variable*/

        void extractYCrCbChannel(Mat input) {
//            "A" channel for detecting red and green by default
            Imgproc.cvtColor(input, duckMat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(duckMat, A, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            extractYCrCbChannel(firstFrame);

            region1_A = A.submat(new Rect(region1_pointA, region1_pointB));
            region2_A = A.submat(new Rect(region2_pointA, region2_pointB));
            region3_A = A.submat(new Rect(region3_pointA, region3_pointB));

            hub_region_left_A = A.submat(new Rect(region1_hub_pointA, region1_hub_pointB));
            hub_region_center_A = A.submat(new Rect(region2_hub_pointA, region2_hub_pointB));
            hub_region_right_A = A.submat(new Rect(region3_hub_pointA, region3_hub_pointB));

        }

        @Override
        public Mat processFrame(Mat input) {


            extractYCrCbChannel(input);
            region1_A = A.submat(new Rect(region1_pointA, region1_pointB));
            region2_A = A.submat(new Rect(region2_pointA, region2_pointB));
            region3_A = A.submat(new Rect(region3_pointA, region3_pointB));

            avg1 = (int) Core.mean(region1_A).val[0];
            avg2 = (int) Core.mean(region2_A).val[0];
            avg3 = (int) Core.mean(region3_A).val[0];

            hub_region_left_A = A.submat(new Rect(region1_hub_pointA, region3_hub_pointB));
            hub_region_center_A = A.submat(new Rect(region2_hub_pointA, region2_hub_pointB));
            hub_region_right_A = A.submat(new Rect(region3_hub_pointA, region3_hub_pointB));

            hub_avg_left_A = (int) Core.mean(hub_region_left_A).val[0];
            hub_avg_center_A = (int) Core.mean(hub_region_center_A).val[0];
            hub_avg_right_A = (int) Core.mean(hub_region_right_A).val[0];

//            hub_region_left_B = B.submat(new Rect(region1_hub_pointA, region3_hub_pointB));
//            hub_region_center_B = B.submat(new Rect(region2_hub_pointA, region2_hub_pointB));
//            hub_region_right_B = B.submat(new Rect(region3_hub_pointA, region3_hub_pointB));

            hub_avg_left_B = (int) Core.mean(hub_region_left_B).val[0];
            hub_avg_center_B = (int) Core.mean(hub_region_center_B).val[0];
            hub_avg_right_B = (int) Core.mean(hub_region_right_B).val[0];


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

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_hub_pointA, // First point which defines the rectangle
                    region1_hub_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    4); // Thickness of the rectangle lines


            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_hub_pointA, // First point which defines the rectangle
                    region2_hub_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    4); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_hub_pointA, // First point which defines the rectangle
                    region3_hub_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    4); // Thickness of the rectangle lines


            if (avg1 < FREIGHT_PRESENT_THRESHOLD) {
                position = DuckPosition.LEFT;
            } else if (avg2 < FREIGHT_PRESENT_THRESHOLD) {
                position = DuckPosition.MIDDLE;
            } else {
                position = DuckPosition.RIGHT;
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

        public int getAnalysisHubLeft() {
            return hub_avg_left_A;
        }

        public int getAnalysisHubCenter() {
            return hub_avg_center_A;
        }

        public int getAnalysisHubRight() {
            return hub_avg_right_A;
        }

    }


}