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

package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the skystone when lined up with
 * the sample regions over the first 3 stones.
 */

    public class TeamElementPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the marker position
         */
        public enum MarkerPosistion
        {
            LEFT,
            CENTER,
            RIGHT,
            UNKNOWN
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(255,108);
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(61,98);

        static final int REGION_WIDTH = 30;
        static final int REGION_HEIGHT = 30;

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
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH+80,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT+10);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                320,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT+50);

        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat HSV = new Mat();
        Mat Cb = new Mat();
        public int avg1, avg2, avg3;
        public static int threshold=110;
        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile MarkerPosistion position = MarkerPosistion.LEFT;


        void inputToHSV(Mat input)
        {
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(HSV, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {

            inputToHSV(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));

        }

        @Override
        public Mat processFrame(Mat input)
        {
           //this makes it all hsv then takes the saturation value so we can see has the most color and stuff
            inputToHSV(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];


            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */


            /*
             * Find the max of the 3 averages
             */
            int max = Math.max(avg1, avg2);


            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if(max == avg1) // Was it from region 1?
            {
                if (max>=threshold) {
                    position = MarkerPosistion.CENTER; // Record our analysis

                    /*
                     * Draw a solid rectangle on top of the chosen region.
                     * Simply a visual aid. Serves no functional purpose.
                     */
                    Imgproc.rectangle(
                            input, // Buffer to draw on
                            region1_pointA, // First point which defines the rectangle
                            region1_pointB, // Second point which defines the rectangle
                            GREEN, // The color the rectangle is drawn in
                            -1); // Negative thickness means solid fill
                }else {position = MarkerPosistion.LEFT;}
            }
            else if(max == avg2) // Was it from region 2?
            {
                if(max>=threshold) {
                    position = MarkerPosistion.RIGHT; // Record our analysis

                    /*
                     * Draw a solid rectangle on top of the chosen region.
                     * Simply a visual aid. Serves no functional purpose.
                     */
                    Imgproc.rectangle(
                            input, // Buffer to draw on
                            region2_pointA, // First point which defines the rectangle
                            region2_pointB, // Second point which defines the rectangle
                            GREEN, // The color the rectangle is drawn in
                            -1); // Negative thickness means solid fill
                }else {position = MarkerPosistion.LEFT;}
            }



            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }
        public double avgzone1(){return avg1;}
        public double avgzone2(){return avg2;}

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public MarkerPosistion getAnalysis()
        {
            return position;
        }
    }
