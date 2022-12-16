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

class PowerPlaySuperPipeline extends OpenCvPipeline
{
    public Object lockBlueCone = new Object();
    public Object lockRedCone = new Object();
    public Object lockPole = new Object();
    boolean detectPole, detectRedCone, detectBlueCone;

    public PowerPlaySuperPipeline(boolean poleDetection, boolean redConeDetection, boolean blueConeDetection)
    {
        detectPole = poleDetection;
        detectRedCone = redConeDetection;
        detectBlueCone = blueConeDetection;
    }

    /*
     * Some stuff to handle returning our various buffers
     */
    enum Stage
    {
        FINAL,
        Cb,
        MASK,
        MASK_NR,
        CONTOURS;
    }

    PoleOrientationExample.PoleOrientationAnalysisPipeline.Stage[] stages = PoleOrientationExample.PoleOrientationAnalysisPipeline.Stage.values();

    // Keep track of what stage the viewport is showing
    int stageNum = 0;

    @Override
    public void onViewportTapped()
    {
        /*
         * Note that this method is invoked from the UI thread
         * so whatever we do here, we must do quickly.
         */

        int nextStageNum = stageNum + 1;

        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }

        stageNum = nextStageNum;
    }

    /*
     * Our working image buffers
     */
    Mat cbMat = new Mat();
    Mat crMat = new Mat();
    Mat thresholdBlueMat = new Mat();
    Mat thresholdRedMat = new Mat();
    Mat thresholdYellowMat = new Mat();
    Mat morphedBlueThreshold = new Mat();
    Mat morphedRedThreshold = new Mat();
    Mat morphedYellowThreshold = new Mat();
    Mat contoursOnPlainImageMat = new Mat();

    /*
     * Threshold values
     */
    static final int CB_CHAN_MASK_YELLOW_THRESHOLD = 80;
    static final int CB_CHAN_MASK_BLUE_THRESHOLD = 160;
    static final int CR_CHAN_MASK_RED_THRESHOLD = 160;

    /*
     * The elements we use for noise reduction
     */
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));

    /*
     * The box constraint that considers a pole "centered"
     */
    RotatedRect CENTERED_OBJECT = new RotatedRect(new double[]{160.0, 120.0, 48.0, 240.0});

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
    static final int CR_CHAN_IDX = 1;

    // This is the allowable distance from the center of the pole to the "center"
    // of the image.  Pole is ~32 pixels wide, so our tolerance is 1/4 of that.
    static final int MAX_POLE_OFFSET = 4;
    static class AnalyzedPole
    {
        RotatedRect corners;
        double centralOffset;
        boolean poleAligned = false;
    }

    static final int MAX_CONE_OFFSET = 4;
    static class AnalyzedCone
    {
        RotatedRect corners;
        double centralOffset;
        boolean coneAligned = false;
    }

    // For detecting poles
    List<AnalyzedPole> internalPoleList = new ArrayList<>();
    volatile List<AnalyzedPole> clientPoleList = Collections.synchronizedList(new ArrayList<>());
    volatile AnalyzedPole thePole = new AnalyzedPole();

    // For detecting red cones
    List<AnalyzedCone> internalRedConeList = new ArrayList<>();
    volatile List<AnalyzedCone> clientRedConeList = Collections.synchronizedList(new ArrayList<>());
    volatile AnalyzedCone theRedCone = new AnalyzedCone();

    // For detecting blue cones
    List<AnalyzedCone> internalBlueConeList = new ArrayList<>();
    volatile List<AnalyzedCone> clientBlueConeList = Collections.synchronizedList(new ArrayList<>());
    volatile AnalyzedCone theBlueCone = new AnalyzedCone();

    @Override
    public Mat processFrame(Mat input)
    {
        // We'll be updating this with new data below
        internalPoleList.clear();
        internalBlueConeList.clear();
        internalRedConeList.clear();

        /*
         * Run the image processing
         *
         * Run analysis for yellow poles
         */
        if(detectPole) {
            synchronized (lockPole) {
                for (MatOfPoint contour : findYellowContours(input)) {
                    AnalyzePoleContour(contour, input);
                }

                clientPoleList.clear();
                if (findThePole()) {
                    clientPoleList.add(thePole);
                }
            }
        }

        /*
         * Run analysis for blue cones
         */
        if(detectBlueCone) {
            synchronized (lockBlueCone) {
                for (MatOfPoint contour : findBlueContours(input)) {
                    AnalyzeBlueConeContour(contour, input);
                }

                clientBlueConeList.clear();
                if (findTheBlueCone()) {
                    clientBlueConeList.add(theBlueCone);
                }
            }
        }

        /*
         * Run analysis for red cones
         */
        if(detectRedCone) {
            synchronized (lockRedCone) {
                for (MatOfPoint contour : findRedContours(input)) {
                    AnalyzeRedConeContour(contour, input);
                }

                clientRedConeList.clear();
                if (findTheRedCone()) {
                    clientRedConeList.add(theRedCone);
                }
            }
        }

        if(detectPole) {
            if (thePole.poleAligned) {
                drawRotatedRect(thePole.corners, input, GREEN);
            } else {
                drawRotatedRect(thePole.corners, input, RED);
            }
        }
        if(detectBlueCone) {
            if (theBlueCone.coneAligned) {
                drawRotatedRect(theBlueCone.corners, input, GREEN);
            } else {
                drawRotatedRect(theBlueCone.corners, input, RED);
            }
        }
        if(detectRedCone) {
            if (theRedCone.coneAligned) {
                drawRotatedRect(theRedCone.corners, input, GREEN);
            } else {
                drawRotatedRect(theRedCone.corners, input, RED);
            }
        }
        drawRotatedRect(CENTERED_OBJECT, input, BLUE);
        /*
         * Decide which buffer to send to the viewport
         */
        switch (stages[stageNum])
        {
            case Cb:
            {
                return cbMat;
            }

            case FINAL:
            {
                return input;
            }

            case MASK:
            {
                return thresholdBlueMat;
            }

            case MASK_NR:
            {
                return morphedBlueThreshold;
            }

            case CONTOURS:
            {
                return contoursOnPlainImageMat;
            }
        }
        return input;
    }

    public List<AnalyzedCone> getDetectedRedCones()
    {
        return Collections.synchronizedList(clientRedConeList);
    }

    List<MatOfPoint> findRedContours(Mat input)
    {
        // A list we'll be using to store the contours we find
        List<MatOfPoint> contoursList = new ArrayList<>();

        // Convert the input image to YCrCb color space, then extract the Cb channel
        Imgproc.cvtColor(input, crMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(crMat, crMat, CR_CHAN_IDX);

        // Threshold the Cr channel to form a mask, then run some noise reduction
        Imgproc.threshold(cbMat, thresholdRedMat, CR_CHAN_MASK_RED_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        morphMask(thresholdRedMat, morphedRedThreshold);

        // Ok, now actually look for the contours! We only look for external contours.
        Imgproc.findContours(morphedRedThreshold, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // We do draw the contours we find, but not to the main input buffer.
        input.copyTo(contoursOnPlainImageMat);
        Imgproc.drawContours(contoursOnPlainImageMat, contoursList, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);

        return contoursList;
    }

    void AnalyzeRedConeContour(MatOfPoint contour, Mat input)
    {
        // Transform the contour to a different format
        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

        // Do a rect fit to the contour, and draw it on the screen
        RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);

        // Make sure it is a cone contour, no need to draw around false pick ups.
        if (isCone(rotatedRectFitToContour))
        {
            AnalyzedCone analyzedCone = new AnalyzedCone();
            analyzedCone.corners = rotatedRectFitToContour;
            analyzedCone.centralOffset = CENTERED_OBJECT.center.x - rotatedRectFitToContour.center.x;
            analyzedCone.coneAligned = abs(analyzedCone.centralOffset) <= MAX_CONE_OFFSET;
            internalRedConeList.add(analyzedCone);
        }
    }

    boolean findTheRedCone()
    {
        boolean foundCone = false;
        theRedCone.centralOffset = 0;
        theRedCone.corners = new RotatedRect(new double[]{0, 0, 0, 0});
        for(AnalyzedCone aCone : internalRedConeList)
        {
            if(aCone.corners.size.height > theRedCone.corners.size.height)
            {
                theRedCone = aCone;
                foundCone = true;
            }
        }
        return foundCone;
    }

    public List<AnalyzedCone> getDetectedBlueCones()
    {
        return Collections.synchronizedList(clientBlueConeList);
    }

    List<MatOfPoint> findBlueContours(Mat input)
    {
        // A list we'll be using to store the contours we find
        List<MatOfPoint> contoursList = new ArrayList<>();

        // Convert the input image to YCrCb color space, then extract the Cb channel
        Imgproc.cvtColor(input, cbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(cbMat, cbMat, CB_CHAN_IDX);

        // Threshold the Cb channel to form a mask, then run some noise reduction
        Imgproc.threshold(cbMat, thresholdBlueMat, CB_CHAN_MASK_BLUE_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        morphMask(thresholdBlueMat, morphedBlueThreshold);

        // Ok, now actually look for the contours! We only look for external contours.
        Imgproc.findContours(morphedBlueThreshold, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        return contoursList;
    }

    void AnalyzeBlueConeContour(MatOfPoint contour, Mat input)
    {
        // Transform the contour to a different format
        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

        // Do a rect fit to the contour, and draw it on the screen
        RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);

        // Make sure it is a cone contour, no need to draw around false pick ups.
        if (isCone(rotatedRectFitToContour))
        {
            AnalyzedCone analyzedCone = new AnalyzedCone();
            analyzedCone.corners = rotatedRectFitToContour;
            analyzedCone.centralOffset = CENTERED_OBJECT.center.x - rotatedRectFitToContour.center.x;
            analyzedCone.coneAligned = abs(analyzedCone.centralOffset) <= MAX_CONE_OFFSET;
            internalBlueConeList.add(analyzedCone);
        }
    }

    boolean findTheBlueCone()
    {
        boolean foundCone = false;
        theBlueCone.centralOffset = 0;
        theBlueCone.corners = new RotatedRect(new double[]{0, 0, 0, 0});
        for(AnalyzedCone aCone : internalBlueConeList)
        {
            if(aCone.corners.size.height > theBlueCone.corners.size.height)
            {
                theBlueCone = aCone;
                foundCone = true;
            }
        }
        return foundCone;
    }

    boolean isCone(RotatedRect rect)
    {
        // We can put whatever logic in here we want to determine the coneness
//        return (rect.size.width > rect.size.height);
        return true;
    }

    public List<AnalyzedPole> getDetectedPoles()
    {
        return Collections.synchronizedList(clientPoleList);
    }

    List<MatOfPoint> findYellowContours(Mat input)
    {
        // A list we'll be using to store the contours we find
        List<MatOfPoint> contoursList = new ArrayList<>();

        // Convert the input image to YCrCb color space, then extract the Cb channel
        Imgproc.cvtColor(input, cbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(cbMat, cbMat, CB_CHAN_IDX);

        // Threshold the Cb channel to form a mask, then run some noise reduction
        Imgproc.threshold(cbMat, thresholdYellowMat, CB_CHAN_MASK_YELLOW_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);
        morphMask(thresholdYellowMat, morphedYellowThreshold);

        // Ok, now actually look for the contours! We only look for external contours.
        Imgproc.findContours(morphedYellowThreshold, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        return contoursList;
    }

    boolean isPole(RotatedRect rect)
    {
        // We can put whatever logic in here we want to determine the poleness
        return (rect.size.width > rect.size.height);
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

    void AnalyzePoleContour(MatOfPoint contour, Mat input)
    {
        // Transform the contour to a different format
        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

        // Do a rect fit to the contour, and draw it on the screen
        RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);

        // Make sure it is a pole contour, no need to draw around false pick ups.
        if (isPole(rotatedRectFitToContour))
        {
            AnalyzedPole analyzedPole = new AnalyzedPole();
            analyzedPole.corners = rotatedRectFitToContour;
            analyzedPole.centralOffset = CENTERED_OBJECT.center.x - rotatedRectFitToContour.center.x;
            analyzedPole.poleAligned = abs(analyzedPole.centralOffset) <= MAX_POLE_OFFSET;
            internalPoleList.add(analyzedPole);
        }
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
