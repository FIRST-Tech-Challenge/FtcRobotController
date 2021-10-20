///*
// * Copyright (c) 2020 OpenFTC Team
// *
// * Permission is hereby granted, free of charge, to any person obtaining a copy
// * of this software and associated documentation files (the "Software"), to deal
// * in the Software without restriction, including without limitation the rights
// * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// * copies of the Software, and to permit persons to whom the Software is
// * furnished to do so, subject to the following conditions:
// *
// * The above copyright notice and this permission notice shall be included in all
// * copies or substantial portions of the Software.
// * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// * SOFTWARE.
// */
//
//package org.firstinspires.ftc.masters;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.internal.camera.names.WebcamNameImpl;
//import org.firstinspires.ftc.robotcore.internal.camera.names.WebcamNameInternal;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//import org.openftc.easyopencv.OpenCvPipeline;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//@TeleOp(name = "TestComputerVision")
//public class EasyOpenCVLocateTeamItem extends LinearOpMode
//{
//    OpenCvWebcam webcam;
//    SkystoneDeterminationPipeline pipeline;
//
//    @Override
//    public void runOpMode()
//    {
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("CameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
////        webcam = OpenCvCameraFactory.getInstance().createWebcam(OpenCvWebcam., cameraMonitorViewId);
//        pipeline = new SkystoneDeterminationPipeline(telemetry);
//        webcam.setPipeline(pipeline);
//
//        // With live preview
//        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//
//        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
//        // out when the RC activity is in portrait. We do our actual image processing assuming
//        // landscape orientation, though.
//        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
//
//
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
//            }
//        });
//
//
//
//        while (opModeIsActive())
//        {
//            telemetry.addData("Analysis1", pipeline.getAnalysis1());
//            telemetry.addData("Analysis2", pipeline.getAnalysis2());
//            telemetry.addData("Analysis3", pipeline.getAnalysis3());
//            telemetry.addData("Position", pipeline.position);
//            telemetry.update();
//
//            // Don't burn CPU cycles busy-looping in this sample
//            sleep(50);
//        }
//    }
//
//    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
//    {
//        Telemetry telemetry;
//        public SkystoneDeterminationPipeline(Telemetry telemetry){
//            this.telemetry= telemetry;
//        }
//
//        /*
//         * An enum to define the freight position
//         */
//        public enum RingPosition
//        {
//                LEFT,
//                MIDDLE,
//                RIGHT
//        }
//
//        /*
//         * Some color constants
//         */
//        static final Scalar BLUE = new Scalar(0, 0, 255);
//        static final Scalar GREEN = new Scalar(0, 255, 0);
//        static final Scalar RED = new Scalar(255,0,0);
//
//        /*
//         * The core values which define the location and size of the sample regions
//         */
//        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(85,176);
//        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(50,176);
//        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(120,176);
//
//
//        static final int REGION_WIDTH = 30;
//        static final int REGION_HEIGHT = 42;
//
//        final int FREIGHT_PRESENT_THRESHOLD = 150;
//
//        Point region1_pointA = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x,
//                REGION1_TOPLEFT_ANCHOR_POINT.y);
//        Point region1_pointB = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//        Point region2_pointA = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x,
//                REGION1_TOPLEFT_ANCHOR_POINT.y);
//        Point region2_pointB = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//        Point region3_pointA = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x,
//                REGION1_TOPLEFT_ANCHOR_POINT.y);
//        Point region3_pointB = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//        /*
//         * Working variables
//         */
//        Mat region1_Cb;
//        Mat region2_Cb;
//        Mat region3_Cb;
//        Mat YCrCb = new Mat();
//        Mat Cb = new Mat();
//        int avg1;
//        int avg2;
//        int avg3;
//
//        // Volatile since accessed by OpMode thread w/o synchronization
//        public volatile RingPosition position = RingPosition.LEFT;
//
//        /*
//         * This function takes the RGB frame, converts to YCrCb,
//         * and extracts the Cb channel to the 'Cb' variable
//         */
//        void inputToCb(Mat input)
//        {
//            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//            Core.extractChannel(YCrCb, Cb, 1);
//        }
//
//        @Override
//        public void init(Mat firstFrame)
//        {
//            inputToCb(firstFrame);
//
//            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
//
//            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
//
//            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
//
//        }
//
//        @Override
//        public Mat processFrame(Mat input)
//        {
//            inputToCb(input);
//
//            avg1 = (int) Core.mean(region1_Cb).val[0];
//            avg2 = (int) Core.mean(region2_Cb).val[0];
//            avg3 = (int) Core.mean(region3_Cb).val[0];
//
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    BLUE, // The color the rectangle is drawn in
//                    2); // Thickness of the rectangle lines
//
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region2_pointA, // First point which defines the rectangle
//                    region2_pointB, // Second point which defines the rectangle
//                    BLUE, // The color the rectangle is drawn in
//                    2); // Thickness of the rectangle lines
//
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region3_pointA, // First point which defines the rectangle
//                    region3_pointB, // Second point which defines the rectangle
//                    BLUE, // The color the rectangle is drawn in
//                    2); // Thickness of the rectangle lines
//
//            position = RingPosition.LEFT; // Record our analysis
//            telemetry.addData("value", avg1);
//            telemetry.update();
//            if(avg1 > FREIGHT_PRESENT_THRESHOLD){
//                position = RingPosition.LEFT;
//            }else if (avg2 > FREIGHT_PRESENT_THRESHOLD){
//                position = RingPosition.MIDDLE;
//            }else if(avg3 > FREIGHT_PRESENT_THRESHOLD){
//                position = RingPosition.RIGHT;
//            }
//
//
//
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    GREEN, // The color the rectangle is drawn in
//                    -1); // Negative thickness means solid fill
//
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region2_pointA, // First point which defines the rectangle
//                    region2_pointB, // Second point which defines the rectangle
//                    GREEN, // The color the rectangle is drawn in
//                    -1); // Negative thickness means solid fill
//
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region3_pointA, // First point which defines the rectangle
//                    region3_pointB, // Second point which defines the rectangle
//                    GREEN, // The color the rectangle is drawn in
//                    -1); // Negative thickness means solid fill
//
//            return input;
//        }
//
//        public int getAnalysis1()
//        {
//            return avg1;
//        }
//        public int getAnalysis2()
//        {
//            return avg2;
//        }
//        public int getAnalysis3()
//        {
//            return avg3;
//        }
//
//
//    }
//
//
//}