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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;


@TeleOp
@Disabled
public class AutoDetectionJunction extends LinearOpMode {
    Hardware robot = new Hardware();
    JunctionDeterminationPipeline pipeline;

    @Override
    public void runOpMode() {

        pipeline = new JunctionDeterminationPipeline();
        robot.webcam2.setPipeline(pipeline);


        robot.webcam2.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        robot.webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Webcam could not be opened", "");
                telemetry.update();
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    public static class JunctionDeterminationPipeline extends OpenCvPipeline {
        /*
         * An enum to define the skystone position
         */
        public enum SignalPosition {
            LEFT,
            CENTER,
            RIGHT
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar PINK = new Scalar(239, 83, 246);
        static final Scalar BLACK = new Scalar(0, 0, 0);



        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION_TOPLEFT_ANCHOR_POINT = new Point(330, 330);
        static final int REGION_WIDTH = 40;
        static final int REGION_HEIGHT = 40;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region_pointA = new Point(
                REGION_TOPLEFT_ANCHOR_POINT.x,
                REGION_TOPLEFT_ANCHOR_POINT.y);
        Point region_pointB = new Point(
                REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region;
        Mat Lab = new Mat();
        Mat L = new Mat();
        Mat a = new Mat();
        public int avgL, avga;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile SignalPosition position = SignalPosition.LEFT;

        /*
         * This function takes the RGB frame, converts to Lab,
         * and extracts the L, a and b channel
         */
//        L – Lightness ( Intensity ).
//        a – color component ranging from Green to Magenta.
//        b – color component ranging from Blue to Yellow.
        void inputToLa(Mat input) {

            Imgproc.cvtColor(input, Lab, Imgproc.COLOR_BGR2RGB /**COLOR_RGB2YCrCb**/);
            Core.extractChannel(Lab, L, 1);
            Core.extractChannel(Lab, a, 2);
        }

        @Override
        public void init(Mat firstFrame) {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToLa(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region = Lab.submat(new Rect(region_pointA, region_pointB));
        }

        public void extractFace(Mat image, int xOne, int xTwo, int yOne, int yTwo) throws CvException {



            Rect rectangle = new Rect(xOne, yOne, xTwo, yTwo);
            Mat result = new Mat();
            Mat bgdModel = new Mat();
            Mat fgdModel = new Mat();
            Mat source = new Mat(1, 1, CvType.CV_8U, new Scalar(3));
            Imgproc.grabCut(image, result, rectangle, bgdModel, fgdModel, 8, Imgproc.GC_INIT_WITH_RECT);
            Core.compare(result, source, result, Core.CMP_EQ);
            Mat foreground = new Mat(image.size(), CvType.CV_8UC3, new Scalar(255, 255, 255));
            image.copyTo(foreground, result);
            //Imgcodecs.imwrite(image, foreground)
        }



        @Override
        public Mat processFrame(Mat input) {

            extractFace(input,200,250,400,450);

            //inputToLa(input);


//            /*
//             * Compute the average pixel value of each submat region. We're
//             * taking the average of a single channel buffer, so the value
//             * we need is at index 0. We could have also taken the average
//             * pixel value of the 3-channel image, and referenced the value
//             * at index 2 here.
//             */
//            avgL = (int) Core.mean(region).val[0];
//            avga = (int) Core.mean(region).val[1];
//            /*
//             * Draw a rectangle showing sample region 1 on the screen.
//             * Simply a visual aid. Serves no functional purpose.
//             */
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region_pointA, // First point which defines the rectangle
//                    region_pointB, // Second point which defines the rectangle
//                    BLUE, // The color the rectangle is drawn in
//                    2); // Thickness of the rectangle lines
//
//            /*
//             * Now that we found the max, we actually need to go and
//             * figure out which sample region that value was from
//             */
//            if (avgL < 100) // Was it from region 1?
//            {
//                position = SignalPosition.CENTER; // Record our analysis
//
//                /*
//                 * Draw a solid rectangle on top of the chosen region.
//                 * Simply a visual aid. Serves no functional purpose.
//                 */
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region_pointA, // First point which defines the rectangle
//                        region_pointB, // Second point which defines the rectangle
//                        BLACK, // The color the rectangle is drawn in
//                        -1); // Negative thickness means solid fill
//            } else if (avga <= 120) // Is it green
//            {
//                position = SignalPosition.LEFT; // Record our analysis
//
//                /*
//                 * Draw a solid rectangle on top of the chosen region.
//                 * Simply a visual aid. Serves no functional purpose.
//                 */
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region_pointA, // First point which defines the rectangle
//                        region_pointB, // Second point which defines the rectangle
//                        GREEN, // The color the rectangle is drawn in
//                        -1); // Negative thickness means solid fill
//            } else if (avga > 120) // Is it Magenta
//            {
//                position = SignalPosition.RIGHT; // Record our analysis
//
//                /*
//                 * Draw a solid rectangle on top of the chosen region.
//                 * Simply a visual aid. Serves no functional purpose.
//                 */
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region_pointA, // First point which defines the rectangle
//                        region_pointB, // Second point which defines the rectangle
//                        PINK, // The color the rectangle is drawn in
//                        -1); // Negative thickness means solid fill
//            }
//
//            /*
//             * Render the 'input' buffer to the viewport. But note this is not
//             * simply rendering the raw camera feed, because we called functions
//             * to add some annotations to this buffer earlier up.
//             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public SignalPosition getAnalysis() {
            return position;
        }
    }
}