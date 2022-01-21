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


public class FreightFrenzyComputerVisionShippingElementReversion{
    OpenCvWebcam webcam;
    public SkystoneDeterminationPipeline pipeline;

    public FreightFrenzyComputerVisionShippingElementReversion (HardwareMap hardwareMap, Telemetry telemetry){
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

        public enum HubPosition {
            LEFT,
            CENTER,
            RIGHT,
            SHRUG_NOISES
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar RED = new Scalar(255, 0, 0);


        static final int HUB_REGION_DISTANCE_FROM_TOP = 33;

        static final int REGION_WIDTH = 30;
        static final int REGION_HEIGHT = 42;

        static final int HUB_REGION_HEIGHT = 50;
        static final int HUB_REGION_WIDTH = 20;

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOP_LEFT_ANCHOR_POINT = new Point(45, 180);
        static final Point REGION2_TOP_LEFT_ANCHOR_POINT = new Point(300, 180);
        static final Point REGION3_TOP_LEFT_ANCHOR_POINT = new Point(575, 180);

        Point[] hubPositionArrayTopLeftPoint= new Point[32];
        Point[] hubPositionArrayBottomRightPoint = new Point[32];

        static final Point HUB_REGION_TOP_LEFT_1 = new Point(0,HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_2 = new Point(HUB_REGION_TOP_LEFT_1.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_3 = new Point(HUB_REGION_TOP_LEFT_2.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_4 = new Point(HUB_REGION_TOP_LEFT_3.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_5 = new Point(HUB_REGION_TOP_LEFT_4.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_6 = new Point(HUB_REGION_TOP_LEFT_5.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_7 = new Point(HUB_REGION_TOP_LEFT_6.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_8 = new Point(HUB_REGION_TOP_LEFT_7.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_9 = new Point(HUB_REGION_TOP_LEFT_8.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_10 = new Point(HUB_REGION_TOP_LEFT_9.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_11 = new Point(HUB_REGION_TOP_LEFT_10.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_12 = new Point(HUB_REGION_TOP_LEFT_11.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_13 = new Point(HUB_REGION_TOP_LEFT_12.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_14 = new Point(HUB_REGION_TOP_LEFT_13.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_15 = new Point(HUB_REGION_TOP_LEFT_14.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_16 = new Point(HUB_REGION_TOP_LEFT_15.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_17 = new Point(HUB_REGION_TOP_LEFT_16.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_18 = new Point(HUB_REGION_TOP_LEFT_17.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_19 = new Point(HUB_REGION_TOP_LEFT_18.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_20 = new Point(HUB_REGION_TOP_LEFT_19.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_21 = new Point(HUB_REGION_TOP_LEFT_20.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_22 = new Point(HUB_REGION_TOP_LEFT_21.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_23 = new Point(HUB_REGION_TOP_LEFT_22.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_24 = new Point(HUB_REGION_TOP_LEFT_23.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_25 = new Point(HUB_REGION_TOP_LEFT_24.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_26 = new Point(HUB_REGION_TOP_LEFT_25.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_27 = new Point(HUB_REGION_TOP_LEFT_26.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_28 = new Point(HUB_REGION_TOP_LEFT_27.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_29 = new Point(HUB_REGION_TOP_LEFT_28.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_30 = new Point(HUB_REGION_TOP_LEFT_29.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_31 = new Point(HUB_REGION_TOP_LEFT_30.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);
        static final Point HUB_REGION_TOP_LEFT_32 = new Point(HUB_REGION_TOP_LEFT_31.x + HUB_REGION_WIDTH, HUB_REGION_DISTANCE_FROM_TOP);



        final int FREIGHT_PRESENT_THRESHOLD = 112;

        final int HUB_PRESENT_THRESHOLD = 129;

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
        Point region_hub_left_pointA = new Point(
                HUB_REGION_TOP_LEFT_1.x,
                HUB_REGION_TOP_LEFT_1.y);
        Point region_hub_left_pointB = new Point(
                HUB_REGION_TOP_LEFT_1.x + 294,
                HUB_REGION_TOP_LEFT_1.y + HUB_REGION_HEIGHT);

        Point region_hub_center_pointA = new Point(
                HUB_REGION_TOP_LEFT_2.x,
                HUB_REGION_TOP_LEFT_2.y);
        Point region_hub_center_pointB = new Point(
                HUB_REGION_TOP_LEFT_2.x + 50,
                HUB_REGION_TOP_LEFT_2.y + HUB_REGION_HEIGHT);

        Point region_hub_right_pointA = new Point(
                HUB_REGION_TOP_LEFT_3.x,
                HUB_REGION_TOP_LEFT_3.y);
        Point region_hub_right_pointB = new Point(
                HUB_REGION_TOP_LEFT_3.x + 294,
                HUB_REGION_TOP_LEFT_3.y + HUB_REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1;
        Mat region2;
        Mat region3;
        Mat LAB = new Mat();

        Mat A = new Mat();
        Mat B = new Mat();

        int avg1 = 0;
        int avg2 = 0;
        int avg3 = 0;

        Mat hub_region_left_A;
        Mat hub_region_center_A;
        Mat hub_region_right_A;



        int hub_avg_left = 0;
        int hub_avg_center = 0;
        int hub_avg_right = 0;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile FreightPosition position = FreightPosition.LEFT;

        public volatile HubPosition hub_position = HubPosition.SHRUG_NOISES;

        /*
         * This function takes the RGB frame, converts to LAB,
         * and extracts the A channel to the 'A' variable*/

        void inputToLAB(Mat input) {

            Imgproc.cvtColor(input, LAB, Imgproc.COLOR_RGB2Lab);
            Core.extractChannel(LAB, A, 1);
            Core.extractChannel(LAB, B, 2);
        }

        @Override
        public void init(Mat firstFrame) {

            hubPositionArrayTopLeftPoint[0] = new Point(0,33);
            hubPositionArrayBottomRightPoint[0] = new Point(hubPositionArrayTopLeftPoint[0].x + 20,hubPositionArrayTopLeftPoint[0].y + 50);

            for (int i=1; i<32; i++){
                hubPositionArrayTopLeftPoint[i]= new Point(hubPositionArrayTopLeftPoint[i-1].x + 20,33);
                hubPositionArrayBottomRightPoint[i] = new Point(hubPositionArrayTopLeftPoint[i-1].x + 20,hubPositionArrayTopLeftPoint[i].y + 50);
            }

            inputToLAB(firstFrame);

            region1 = A.submat(new Rect(region1_pointA, region1_pointB));
            region2 = A.submat(new Rect(region2_pointA, region2_pointB));
            region3 = A.submat(new Rect(region3_pointA, region3_pointB));

            hub_region_left_A = A.submat(new Rect(region_hub_left_pointA, region_hub_left_pointB));
            hub_region_center_A = A.submat(new Rect(region_hub_center_pointA, region_hub_center_pointB));
            hub_region_right_A = A.submat(new Rect(region_hub_right_pointA, region_hub_right_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToLAB(input);
            region1 = A.submat(new Rect(region1_pointA, region1_pointB));
            region2 = A.submat(new Rect(region2_pointA, region2_pointB));
            region3 = A.submat(new Rect(region3_pointA, region3_pointB));

            avg1 = (int) Core.mean(region1).val[0];
            avg2 = (int) Core.mean(region2).val[0];
            avg3 = (int) Core.mean(region3).val[0];



            hub_region_left_A = A.submat(new Rect(region_hub_left_pointA, region_hub_right_pointB));
            hub_region_center_A = A.submat(new Rect(region_hub_center_pointA, region_hub_center_pointB));
            hub_region_right_A = A.submat(new Rect(region_hub_right_pointA, region_hub_right_pointB));

            Mat[] hubRegionRed = new Mat[32];
            Mat[] hubRegionBlue = new Mat[32];

            for (int i = 0; i<32; i++) {
                hubRegionRed[i] = A.submat(new Rect(hubPositionArrayTopLeftPoint[i], hubPositionArrayBottomRightPoint[i]));
                hubRegionBlue[i] = B.submat(new Rect(hubPositionArrayTopLeftPoint[i], hubPositionArrayBottomRightPoint[i]));
            }

            int[] hubRegionAvgRed = new int[32];
            int[] hubRegionAvgBlue = new int[32];

            for (int i = 0; i<32; i++) {
                hubRegionAvgRed[i] = (int) Core.mean(hubRegionRed[i]).val[0];
                hubRegionAvgBlue[i] = (int) Core.mean(hubRegionBlue[i]).val[0];
            }

            hub_avg_left = (int) Core.mean(hub_region_left_A).val[0];
            hub_avg_center = (int) Core.mean(hub_region_center_A).val[0];
            hub_avg_right = (int) Core.mean(hub_region_right_A).val[0];

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

            for (int i = 0; i<32; i++) {
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        hubPositionArrayTopLeftPoint[i], // First point which defines the rectangle
                        hubPositionArrayBottomRightPoint[i], // Second point which defines the rectangle
                        BLUE, // The color the rectangle is drawn in
                        2); // Thickness of the rectangle lines

            }

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

            if (hub_avg_left >= HUB_PRESENT_THRESHOLD) {
                hub_position = HubPosition.LEFT;
            } else if (hub_avg_center >= HUB_PRESENT_THRESHOLD) {
                hub_position = HubPosition.CENTER;
            } else {
                hub_position = HubPosition.RIGHT;
            }

            int maxRedIndex = 0;


//            Nonfunctional! Logic problem not a syntax one.
            for (int i = 0; i<32; i++) {
                if (hubRegionAvgRed[i] > hubRegionAvgRed[maxRedIndex]) {
                    maxRedIndex = i;
                }
            }

            telemetry.addData("Hub Red Center Average", hubRegionAvgRed[16]);
            telemetry.addData("Hub Blue Center Average", hubRegionAvgBlue[16]);
            telemetry.addData("Hub Blue Center Average", (int) Core.mean(hubRegionBlue[16]).val[0]);

//            telemetry.addData("Analysis of Hub Left", hub_avg_left);
//            telemetry.addData("Analysis2 of Hub Center", hub_avg_center);
//            telemetry.addData("Analysis3 of Hub Right", hub_avg_right);
//            telemetry.addData("Position", hub_position);


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
            return hub_avg_left;
        }

        public int getAnalysisHubCenter() {
            return hub_avg_center;
        }

        public int getAnalysisHubRight() {
            return hub_avg_right;
        }

    }


}