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

import static java.lang.Math.abs;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

    class PoleOrientationPipeline extends OpenCvPipeline
    {
        /*
         * Our working image buffers
         */
        Mat cbMat = new Mat();
        Mat thresholdMat = new Mat();
        Mat morphedThreshold = new Mat();
        Mat contoursOnPlainImageMat = new Mat();

        /*
         * Threshold values
         */
        static final int CB_CHAN_MASK_THRESHOLD = 80;
        static final double DENSITY_UPRIGHT_THRESHOLD = 0.03;

        /*
         * The elements we use for noise reduction
         */
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));

        /*
         * The box constraint that considers a pole "centered"
         */
        RotatedRect CENTERED_POLE = new RotatedRect(new double[]{160.0, 120.0, 48.0, 240.0});

        /*
         * Colors
         */
        static final Scalar TEAL = new Scalar(3, 148, 252);
        static final Scalar PURPLE = new Scalar(158, 52, 235);
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar BLUE = new Scalar(0, 0, 255);

        static final int CONTOUR_LINE_THICKNESS = 2;
        static final int CB_CHAN_IDX = 2;

        // This is the allowable distance from the center of the pole to the "center"
        // of the image.  Pole is ~32 pixels wide, so our tolerance is 1/4 of that.
        static final int MAX_POLE_OFFSET = 8;
        static class AnalyzedPole
        {
            RotatedRect corners;
            double centralOffset;
            boolean poleAligned = false;
        }

        ArrayList<AnalyzedPole> internalPoleList = new ArrayList<>();
        volatile List<AnalyzedPole> clientPoleList = Collections.synchronizedList(new ArrayList<AnalyzedPole>());
        volatile AnalyzedPole thePole = new AnalyzedPole();

        @Override
        public Mat processFrame(Mat input)
        {
            // We'll be updating this with new data below
            internalPoleList.clear();
            drawRotatedRect(CENTERED_POLE, input, BLUE);

            /*
             * Run the image processing
             */
            for(MatOfPoint contour : findContours(input))
            {
                analyzeContour(contour, input);
            }

            clientPoleList.clear();
            if(findThePole())
            {
                clientPoleList.add(thePole);
                if (abs(thePole.centralOffset) <= MAX_POLE_OFFSET)
                {
                    drawRotatedRect(thePole.corners, input, GREEN);
                    thePole.poleAligned = true;
                } else {
                    drawRotatedRect(thePole.corners, input, RED);
                    thePole.poleAligned = false;
                }
            }

            return input;
        }

        public ArrayList<AnalyzedPole> getDetectedPoles()
        {
            return new ArrayList<>(clientPoleList);
        }

        ArrayList<MatOfPoint> findContours(Mat input)
        {
            // A list we'll be using to store the contours we find
            ArrayList<MatOfPoint> contoursList = new ArrayList<>();

            // Convert the input image to YCrCb color space, then extract the Cb channel
            Imgproc.cvtColor(input, cbMat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(cbMat, cbMat, CB_CHAN_IDX);

            // Threshold the Cb channel to form a mask, then run some noise reduction
            Imgproc.threshold(cbMat, thresholdMat, CB_CHAN_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);
            morphMask(thresholdMat, morphedThreshold);

            // Ok, now actually look for the contours! We only look for external contours.
            Imgproc.findContours(morphedThreshold, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

            // We do draw the contours we find, but not to the main input buffer.
            input.copyTo(contoursOnPlainImageMat);
            Imgproc.drawContours(contoursOnPlainImageMat, contoursList, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);

            return contoursList;
        }

        void morphMask(Mat input, Mat output)
        {
            /*
             * Apply some erosion and dilation for noise reduction
             */
            Imgproc.erode(input, output, erodeElement);
            Imgproc.erode(output, output, erodeElement);

            Imgproc.dilate(output, output, dilateElement);
            Imgproc.dilate(output, output, dilateElement);
        }

        boolean isPole(RotatedRect rect)
        {
            // We can put whatever logic in here we want to determine the poleness
            return (rect.size.width > rect.size.height);
//            return true;
        }

        boolean findThePole()
        {
            boolean foundPole = false;
            thePole.centralOffset = 0;
            thePole.corners = new RotatedRect(new double[]{0, 0, 0, 0});
            for(AnalyzedPole aPole : internalPoleList)
            {
                if(aPole.corners.size.height > thePole.corners.size.height)
                {
                    thePole = aPole;
                    foundPole = true;
                }
            }
            return foundPole;
        }

        void analyzeContour(MatOfPoint contour, Mat input)
        {
            // Transform the contour to a different format
            Point[] points = contour.toArray();
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            // Do a rect fit to the contour, and draw it on the screen
            RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);

            // Make sure it is a pole contour, no need to draw around false pick ups.
            if (isPole(rotatedRectFitToContour))
            {
                AnalyzedPole analyzedPole = new AnalyzedPole();
                analyzedPole.corners = rotatedRectFitToContour;
                analyzedPole.centralOffset = CENTERED_POLE.center.x - rotatedRectFitToContour.center.x;
                internalPoleList.add(analyzedPole);
            }
       }

       static void drawRotatedRect(RotatedRect rect, Mat drawOn, Scalar color)
       {
           /*
            * Draws a rotated rect by drawing each of the 4 lines individually
            */
           Point[] points = new Point[4];
           rect.points(points);

           for(int i = 0; i < 4; ++i)
           {
               Imgproc.line(drawOn, points[i], points[(i+1)%4], color, 2);
           }
       }
    }
