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
//import android.os.Build;
//
//import androidx.annotation.RequiresApi;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//import java.util.ArrayList;
//import java.util.List;
//import java.util.NoSuchElementException;
//
//@TeleOp(name="The Mighty Fragrances' Favorite Uncle")
//public class ObsoleteEasyOpenCVExample extends LinearOpMode
//{
//    OpenCvWebcam duckWebcam;
//    WarehousePipeline pipeline;
//    @Override
//    public void runOpMode()
//    {
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        duckWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
//        pipeline = new WarehousePipeline(telemetry);
//        duckWebcam.setPipeline(pipeline);
//
//        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
//        // out when the RC activity is in portrait. We do our actual image processing assuming
//        // landscape orientation, though.
//        duckWebcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
//
//        duckWebcam.setMillisecondsPermissionTimeout(2500);
//        duckWebcam.setMillisecondsPermissionTimeout(2500);
//
//        // Open both camera streams. Closing unnecessary streams mid autonomous is possible and likely recommended.
//        duckWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                /*
//                 * Tell the webcam to start streaming images to us! Note that you must make sure
//                 * the resolution you specify is supported by the camera. If it is not, an exception
//                 * will be thrown.
//                 *
//                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
//                 * supports streaming from the webcam in the uncompressed YUV image format. This means
//                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
//                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
//                 *
//                 * Also, we specify the rotation that the webcam is used in. This is so that the image
//                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
//                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
//                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
//                 * away from the user.
//                 */
//                telemetry.addData("Duck Webcam Opened", "Yes");
//                duckWebcam.setPipeline(pipeline);
//                duckWebcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
//                telemetry.addData("duck webcam startStreaming", "yes");
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addLine("Can't open duck cameras");
//                telemetry.update();
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });
//
//        waitForStart();
//
//        while (opModeIsActive())
//        {
//            telemetry.addData("Position", pipeline.freightPosition);
//
//            // Don't burn CPU cycles busy-looping in this sample
//            sleep(50);
//        }
//    }
//
//    public static class WarehousePipeline extends OpenCvPipeline{
//        Telemetry telemetry;
//        public WarehousePipeline(Telemetry telemetry) {
//            this.telemetry = telemetry;
//        }
//
//        //    All possible regions to be detected in are stored in an enum
//        public enum FreightPosition {
//            LEFT5,
//            LEFT4,
//            LEFT3,
//            LEFT2,
//            LEFT1,
//            CENTER,
//            RIGHT1,
//            RIGHT2,
//            RIGHT3,
//            RIGHT4,
//            RIGHT5,
//            SHRUG_NOISES
//        }
//
//        public enum FreightLocation {
//            NEAR,
//            MIDDLE,
//            FAR,
//            UNKNOWN
//        }
//
//
//
//        /*
//         * Some color constants used for displaying rectangles on the camera stream
//         */
//        static final Scalar BLUE = new Scalar(0, 0, 255);
//        static final Scalar GREEN = new Scalar(0, 255, 0);
//        static final Scalar RED = new Scalar(255, 0, 0);
//
//
//        //        Sizes for subregions of the camera from which our data is extracted
//        static final int REGION_WIDTH = 120;
//        static final int REGION_HEIGHT = 120;
//
//        /*
//         * List for the storage of points, if you're only dealing with a few regions declare them all seperately, the freight regions in the other pipeline
//         * are done like this.
//         */
//
//        ArrayList<Point> topLeftPoints = new ArrayList<Point>();
//        ArrayList<Point> bottomRightPoints = new ArrayList<Point>();
//
//
//        //        The threshold to which the averages are compared.
//        final int DUCK_PRESENT_THRESHOLD = 160;
//
//
//
//        /*
//         * Empty matrixes that data will be stored in
//         */
//
//        Mat LAB = new Mat();
//        Mat A = new Mat();
//        Mat B = new Mat();
//
//        // Volatile since accessed by OpMode thread w/o synchronization
//        public volatile FreightPosition freightPosition = FreightPosition.CENTER;
//        public volatile FreightLocation freightDistance = FreightLocation.UNKNOWN;
//        /*
//         * This function takes the RGB frame, converts to LAB,
//         * and extracts the A channel to the 'A' variable
//         */
//        void inputToLAB(Mat input) {
//
//            Imgproc.cvtColor(input, LAB, Imgproc.COLOR_RGB2Lab);
//            Core.extractChannel(LAB, A, 1);
//            Core.extractChannel(LAB, B, 2);
//        }
//
//        //        Done in innit, this was a more complicated formation of subregions for detecting the duck, but essentially
////        just assign the top left and bottom right points for each region you desire.
//        @Override
//        public void init(Mat firstFrame) {
//
//            topLeftPoints.add(new Point(20,1));
//            bottomRightPoints.add(new Point(topLeftPoints.get(0).x + REGION_WIDTH, topLeftPoints.get(0).y + REGION_HEIGHT));
//
//            int tempY = 0;
//            int tempX = 20;
//            for (int i = 1; i < 27; i++) {
//                if (tempX == 500) {
//                    tempX = 20;
//                    tempY += REGION_HEIGHT;
//                } else {
//                    tempX += REGION_WIDTH/2;
//                }
//                topLeftPoints.add(new Point(tempX,tempY));
//                bottomRightPoints.add(new Point(tempX + REGION_WIDTH, tempY + REGION_HEIGHT));
//            }
//
//            inputToLAB(firstFrame);
//
//
//        }
//
//        //        Process frame, takes a matrix input and processes it. I still have no idea WHERE this is called, but it is absolutely essential to CV functioning
//        @RequiresApi(api = Build.VERSION_CODES.N)
//        @Override
//        public Mat processFrame(Mat input) {
//            inputToLAB(input);
//
////            Declare list of regions
//            ArrayList<Mat> region = new ArrayList<Mat>();
//
//
////            Put the necessary data from the frame to each region
//            for (int i = 0; i<27; i++) {
//                region.add(B.submat(new Rect(topLeftPoints.get(i), bottomRightPoints.get(i))));
//            }
//
//            ArrayList<Integer> regionAvgs = new ArrayList<Integer>();
//
//            for (int i = 0; i<27; i++) {
//                regionAvgs.add((int) Core.mean(region.get(i)).val[0]);
//            }
//
////              This is what displays the rectangles to the camera stream on the drive hub
//            for (int i = 0; i<27; i++) {
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        topLeftPoints.get(i), // First point which defines the rectangle
//                        bottomRightPoints.get(i), // Second point which defines the rectangle
//                        BLUE, // The color the rectangle is drawn in
//                        2); // Thickness of the rectangle lines
//
//            }
//
////            The next several lines determine which region has the highest B average.
//            int indexOfMaximumBAvg = 0;
//            int maxIndexDistance = 2794;
//
//            for (int i = 0; i<27; i++) {
//                if (regionAvgs.get(i) >= regionAvgs.get(indexOfMaximumBAvg)) {
//                    indexOfMaximumBAvg = i;
//                }
//            }
//
////            Display telemetry
//            telemetry.addData("B Averages", regionAvgs);
//            telemetry.addData("Index of highest likelihood.", indexOfMaximumBAvg);
//
////            Fix indexes (Regions are stacked 11 to a row)
////            if (regionAvgs.get(indexOfMaximumBAvg) >= DUCK_PRESENT_THRESHOLD) {
////                if (indexOfMaximumBAvg > 21) {
////                    indexOfMaximumBAvg -= 22;
////                    maxIndexDistance = 0;
////                } else if (indexOfMaximumBAvg > 10) {
////                    indexOfMaximumBAvg -= 11;
////                    maxIndexDistance = 1;
////                } else {
////                    maxIndexDistance = 0;
////                }
////                position = MultipleCameraCV.DuckDeterminationPipeline.DuckPosition.values()[indexOfMaximumBAvg];
////            } else {
////                position = MultipleCameraCV.DuckDeterminationPipeline.DuckPosition.SHRUG_NOISES; // Default enum result. Aptly named
////            }
//            List<Integer> nearRow = (List<Integer>) regionAvgs.subList(18,27);
//            List<Integer> middleRow = (List<Integer>) regionAvgs.subList(9,18);
//            List<Integer> farRow = (List<Integer>) regionAvgs.subList(0,9);
//
//
//
//            if (nearRow.stream().mapToInt(v -> v).max().orElseThrow(NoSuchElementException::new) > DUCK_PRESENT_THRESHOLD) {
//                indexOfMaximumBAvg = 0;
//                for (int i = 0; i<9; i++) {
//                    if (nearRow.get(i) >= regionAvgs.get(indexOfMaximumBAvg)) {
//                        indexOfMaximumBAvg = i;
//                    }
//                }
//                freightPosition = FreightPosition.values()[indexOfMaximumBAvg];
//                freightDistance = FreightLocation.NEAR;
//            } else if (middleRow.stream().mapToInt(v -> v).max().orElseThrow(NoSuchElementException::new) > DUCK_PRESENT_THRESHOLD) {
//                indexOfMaximumBAvg = 0;
//                for (int i = 0; i<9; i++) {
//                    if (nearRow.get(i) >= regionAvgs.get(indexOfMaximumBAvg)) {
//                        indexOfMaximumBAvg = i;
//                    }
//                }
//                freightPosition = FreightPosition.values()[indexOfMaximumBAvg];
//                freightDistance = FreightLocation.MIDDLE;
//            } else if (farRow.stream().mapToInt(v -> v).max().orElseThrow(NoSuchElementException::new) > DUCK_PRESENT_THRESHOLD) {
//                indexOfMaximumBAvg = 0;
//                for (int i = 0; i<9; i++) {
//                    if (nearRow.get(i) >= regionAvgs.get(indexOfMaximumBAvg)) {
//                        indexOfMaximumBAvg = i;
//                    }
//                }
//                freightPosition = FreightPosition.values()[indexOfMaximumBAvg];
//                freightDistance = FreightLocation.FAR;
//            } else {
//                freightPosition = FreightPosition.SHRUG_NOISES;
//            }
//
//
//            telemetry.addData("Position", freightPosition);
//            telemetry.addData("Distance", freightDistance);
//            telemetry.update();
//
//            return input;
//        }
//
//
//    }
//
//
//}