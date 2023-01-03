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

import static org.firstinspires.ftc.teamcode.PowerPlaySuperPipeline.DebugObjects.Pole;
import static java.lang.Math.abs;

import android.os.Environment;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

class PowerPlaySuperPipeline extends OpenCvPipeline
{
    // Variables for the signal detection
    ArrayList<Mat> channels = new ArrayList<>(3);
    private Mat r    = new Mat();
    private Mat g    = new Mat();
    private Mat b    = new Mat();
    private int max;
    public int avgR;
    public int avgG;
    public int avgB;
    private Point marker = new Point();        // Team Element (populated once we find it!)
    private Point sub1PointA;
    private Point sub1PointB;

    // Public statics to be used by opMode
    public int signalZone;

    public Mat finalAutoImage = new Mat();

    private final boolean blueAlliance;
    private final boolean terminalSide;
    private static String directory;

    public Object lockBlueTape = new Object();
    public Object lockRedTape = new Object();
    public Object lockBlueCone = new Object();
    public Object lockRedCone = new Object();
    public Object lockPole = new Object();
    boolean detectPole, detectRedCone, detectBlueCone, detectSignal;

    public PowerPlaySuperPipeline(boolean signalDetection, boolean poleDetection,
                                  boolean redConeDetection, boolean blueConeDetection,
                                  double center, boolean blueAlliance, boolean terminalSide)
    {
        detectSignal = signalDetection;
        detectPole = poleDetection;
        detectRedCone = redConeDetection;
        detectBlueCone = blueConeDetection;
        CENTERED_OBJECT.center.y = center;
        this.blueAlliance = blueAlliance;
        this.terminalSide = terminalSide;
        /*
         * We won't turn on signal detection after we start the pipeline, so shouldn't be a problem
         * to only create the directories in the constructor.
         */
        if(detectSignal) {
            // Create a subdirectory based on DATE
            String dateString = new SimpleDateFormat("yyyy-MM-dd", Locale.getDefault()).format(new Date());
            directory = Environment.getExternalStorageDirectory().getPath() + "//FIRST//Webcam//" + dateString;
            if (this.blueAlliance) {
                if (this.terminalSide) {
                    directory += "/blue_terminal";
                } else {
                    directory += "/blue_not_terminal";
                }
            } else {
                if (this.terminalSide) {
                    directory += "/red_terminal";
                } else {
                    directory += "/red_not_terminal";
                }
            }
            sub1PointA = new Point(212, 89);  // 20x39 pixel box for signal sleeve
            sub1PointB = new Point(231, 127);

            // Create the directory structure to store the autonomous image used to start auto.
            File autonomousDir = new File(directory);
            autonomousDir.mkdirs();
        }
    }

    /*
     * Turn on/off detecting the signal cone
     */
    public void signalDetection(boolean enabled) {
        detectSignal = enabled;
    }

    /*
     * Turn on/off detecting the blue cone
     */
    public void blueConeDetection(boolean enabled) {
        detectBlueCone = enabled;
    }

    /*
     * Turn on/off detecting the blue cone
     */
    public void redConeDetection(boolean enabled) {
        detectRedCone = enabled;
    }

    /*
     * Turn on/off detecting the pole
     */
    public void poleDetection(boolean enabled) {
        detectPole = enabled;
    }

    public void saveLastAutoImage() {
        String timeString = new SimpleDateFormat("hh-mm-ss", Locale.getDefault()).format(new Date());
        String directoryPath = directory + "/" + "AutoImage_" + timeString + ".png";

        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                try
                {
                    Imgproc.cvtColor(finalAutoImage, finalAutoImage, Imgproc.COLOR_RGB2BGR);
                    Imgcodecs.imwrite(directoryPath, finalAutoImage);
                }
                catch (Exception e)
                {
                    e.printStackTrace();
                }
                finally
                {
                    finalAutoImage.release();
                }
            }
        }).start();
    }

    /*
     * Some stuff to handle returning our various buffers
     */
    enum Stage
    {
        FINAL,
        Cx,
        MASK,
        MASK_NR,
        CONTOURS,
        RECTANGLES;
    }

    enum DebugObjects
    {
        Pole,
        ConeRed,
        ConeBlue
    }
    Stage[] stages = Stage.values();
    public DebugObjects debugType = Pole;

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
    Mat cyMat = new Mat();
    Mat thresholdBlueMat = new Mat();
    Mat thresholdRedMat = new Mat();
    Mat thresholdYellowMat = new Mat();
    Mat morphedBlueThreshold = new Mat();
    Mat morphedRedThreshold = new Mat();
    Mat morphedYellowThreshold = new Mat();
    Mat contoursOnPlainImageMat = new Mat();
    Mat rectanglesOnPlainImageMat = new Mat();

    /*
     * Threshold values
     */
    static final int CB_CHAN_MASK_YELLOW_THRESHOLD = 80;
    static final int CB_CHAN_MASK_BLUE_THRESHOLD = 140;
    static final int CR_CHAN_MASK_RED_THRESHOLD = 140;

    /*
     * The elements we use for noise reduction
     */
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(1, 3));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2, 6));

    /*
     * The box constraint that considers a pole "centered"
     */
    RotatedRect CENTERED_OBJECT = new RotatedRect(new double[]{160.0, 120.0, 48.0, 320.0, 0.0});

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
    // This  is how wide a pole is at the proper scoring distance on a high pole
    static final int POLE_HIGH_DISTANCE = 32;
    // This is how many pixels wide the pole can vary at the proper scoring distance
    // on a high pole.
    static final int MAX_HIGH_DISTANCE_OFFSET = 3;
    static class AnalyzedPole
    {
        public AnalyzedPole() {};
        public AnalyzedPole(AnalyzedPole copyObject) {
            corners = copyObject.corners.clone();
            alignedCount = copyObject.alignedCount;
            properDistanceHighCount = copyObject.properDistanceHighCount;
            centralOffset = copyObject.centralOffset;
            highDistanceOffset = copyObject.highDistanceOffset;
            aligned = copyObject.aligned;
            properDistanceHigh = copyObject.properDistanceHigh;
        }
        RotatedRect corners;
        int alignedCount = 0;
        int properDistanceHighCount = 0;
        double centralOffset;
        double highDistanceOffset;
        boolean aligned = false;
        boolean properDistanceHigh = false;
    }

    static final int MAX_CONE_OFFSET = 4;
    static class AnalyzedCone
    {
        public AnalyzedCone() {};
        public AnalyzedCone(AnalyzedCone copyObject) {
            corners = copyObject.corners.clone();
            alignedCount = copyObject.alignedCount;
            centralOffset = copyObject.centralOffset;
            aligned = copyObject.aligned;
        }
        RotatedRect corners;
        int alignedCount = 0;
        double centralOffset;
        boolean aligned = false;
    }

    static final int MAX_TAPE_OFFSET = 4;
    static class AnalyzedTape
    {
        public AnalyzedTape() {};
        public AnalyzedTape(AnalyzedTape copyObject) {
            corners = copyObject.corners.clone();
            alignedCount = copyObject.alignedCount;
            centralOffset = copyObject.centralOffset;
            aligned = copyObject.aligned;
            angle = copyObject.angle;
        }
        RotatedRect corners;
        int alignedCount = 0;
        double centralOffset;
        boolean aligned = false;
        double angle = 0.0;
    }

    // For detecting poles
    List<AnalyzedPole> internalPoleList = new ArrayList<>();
    AnalyzedPole thePole = new AnalyzedPole();

    // For detecting red cones
    List<AnalyzedCone> internalRedConeList = new ArrayList<>();
    AnalyzedCone theRedCone = new AnalyzedCone();

    // For detecting blue cones
    List<AnalyzedCone> internalBlueConeList = new ArrayList<>();
    AnalyzedCone theBlueCone = new AnalyzedCone();

    // For detecting red tape
    List<AnalyzedTape> internalRedTapeList = new ArrayList<>();
    AnalyzedTape theRedTape = new AnalyzedTape();

    // For detecting blue tape
    List<AnalyzedTape> internalBlueTapeList = new ArrayList<>();
    AnalyzedTape theBlueTape = new AnalyzedTape();

    @Override
    public Mat processFrame(Mat input)
    {
        // We do draw the contours we find, but not to the main input buffer.
        input.copyTo(contoursOnPlainImageMat);
        input.copyTo(rectanglesOnPlainImageMat);

        /*
         * Run the image processing
         */
        /*
         * Run analysis for signal detection
         */
        if(detectSignal) {
            // Extract the RGB channels from the image frame
            Core.split(input, channels);

            // Pull RGB data for the sample zone from the RBG channels
            r = channels.get(0).submat(new Rect(sub1PointA,sub1PointB) );
            g = channels.get(1).submat(new Rect(sub1PointA,sub1PointB) );
            b = channels.get(2).submat(new Rect(sub1PointA,sub1PointB) );

            // Average the three sample zones
            avgR = (int)Core.mean(r).val[0];
            avgB = (int)Core.mean(b).val[0];
            avgG = (int)Core.mean(g).val[0];

            // Determine which RBG channel had the highest value
            max = Math.max(avgR, Math.max(avgB, avgG));
            // Draw a circle on the detected team shipping element
            marker.x = (sub1PointA.x + sub1PointB.x) / 2;
            marker.y = (sub1PointA.y + sub1PointB.y) / 2;

            // Free the allocated submat memory
            r.release();
            r = null;
            g.release();
            g = null;
            b.release();
            b = null;
            channels.get(0).release();
            channels.get(1).release();
            channels.get(2).release();
        }

        /*
         * Run analysis for yellow poles
         */
        if(detectPole) {
            internalPoleList.clear();

            for (MatOfPoint contour : findYellowContours(input)) {
                AnalyzePoleContour(contour, input);
            }

            drawPoleRects(internalPoleList, rectanglesOnPlainImageMat, BLUE);

            findThePole();
        }

        /*
         * Run analysis for blue cones
         */
        if(detectBlueCone) {
            internalBlueTapeList.clear();
            internalBlueConeList.clear();
            findBlueContours(input);

            drawConeRotatedRects(internalBlueConeList, rectanglesOnPlainImageMat, BLUE);
            drawTapeRotatedRects(internalBlueTapeList, rectanglesOnPlainImageMat, BLUE);

            findTheBlueCone();
            findTheBlueTape();
        }

        /*
         * Run analysis for red cones
         */
        if(detectRedCone) {
            internalRedTapeList.clear();
            internalRedConeList.clear();
            findRedContours(input);

            drawConeRotatedRects(internalRedConeList, rectanglesOnPlainImageMat, BLUE);
            drawTapeRotatedRects(internalRedTapeList, rectanglesOnPlainImageMat, BLUE);

            findTheRedCone();
            findTheRedTape();
        }

        // Above here, do not draw on input. All the drawing is done below here so it is not part
        // of the processing.
        if(detectSignal) {
            // Draw rectangles around the sample zone
            Imgproc.rectangle(input, sub1PointA, sub1PointB, new Scalar(0, 0, 255), 1);
            if(max == avgR) {
                Imgproc.circle(input, marker, 5, new Scalar(255, 0, 0), -1);
                signalZone = 1;
            } else if(max == avgG) {
                Imgproc.circle(input, marker, 5, new Scalar(0, 255, 0), -1);
                signalZone = 2;
            } else if(max == avgB) {
                Imgproc.circle(input, marker, 5, new Scalar(0, 0, 255), -1);
                signalZone = 3;
            } else {
                signalZone = 3;
            }
            input.copyTo(finalAutoImage);
        }

        if(detectPole) {
            synchronized(lockPole) {
                if (thePole.aligned) {
                    thePole.alignedCount++;
                    drawRotatedRect(thePole.corners, input, GREEN);
                } else {
                    thePole.alignedCount = 0;
                    drawRotatedRect(thePole.corners, input, RED);
                }
                if(thePole.properDistanceHigh) {
                    thePole.properDistanceHighCount++;
                } else {
                    thePole.properDistanceHighCount = 0;
                }
            }
        }
        if(detectBlueCone) {
            synchronized(lockBlueCone) {
                if (theBlueCone.aligned) {
                    theBlueCone.alignedCount++;
                    drawRotatedRect(theBlueCone.corners, input, GREEN);
                } else {
                    theBlueCone.alignedCount = 0;
                    drawRotatedRect(theBlueCone.corners, input, RED);
                }
            }
            synchronized(lockBlueTape) {
                if (theBlueTape.aligned) {
                    theBlueTape.alignedCount++;
                    drawRotatedRect(theBlueTape.corners, input, GREEN);
                } else {
                    theBlueTape.alignedCount = 0;
                    drawRotatedRect(theBlueTape.corners, input, RED);
                }
            }
        }
        if(detectRedCone) {
            synchronized(lockRedCone) {
                if (theRedCone.aligned) {
                    theRedCone.alignedCount++;
                    drawRotatedRect(theRedCone.corners, input, GREEN);
                } else {
                    theRedCone.alignedCount = 0;
                    drawRotatedRect(theRedCone.corners, input, RED);
               }
            }
            synchronized(lockRedTape) {
                if (theRedTape.aligned) {
                    theRedTape.alignedCount++;
                    drawRotatedRect(theRedTape.corners, input, GREEN);
                } else {
                    theRedTape.alignedCount = 0;
                    drawRotatedRect(theRedTape.corners, input, RED);
                }
            }
        }
        drawRotatedRect(CENTERED_OBJECT, input, BLUE);
        /*
         * Decide which buffer to send to the viewport
         */
        switch (stages[stageNum])
        {
            case Cx:
            {
                switch(debugType) {
                    case Pole:
                        return cyMat;
                    case ConeRed:
                        return crMat;
                    case ConeBlue:
                        return cbMat;
                }
            }

            case FINAL:
            {
                return input;
            }

            case MASK:
            {
                switch(debugType) {
                    case Pole:
                        return thresholdYellowMat;
                    case ConeRed:
                        return thresholdRedMat;
                    case ConeBlue:
                        return thresholdBlueMat;
                }
            }

            case MASK_NR:
            {
                switch(debugType) {
                    case Pole:
                        return morphedYellowThreshold;
                    case ConeRed:
                        return morphedRedThreshold;
                    case ConeBlue:
                        return morphedBlueThreshold;
                }
            }

            case CONTOURS:
            {
                return contoursOnPlainImageMat;
            }

            case RECTANGLES:
            {
                return rectanglesOnPlainImageMat;
            }
        }
        return input;
    }

    public AnalyzedCone getDetectedRedCone()
    {
        synchronized(lockRedCone) {
            return new AnalyzedCone(theRedCone);
        }
    }

    public AnalyzedTape getDetectedRedTape()
    {
        synchronized(lockRedTape) {
            return new AnalyzedTape(theRedTape);
        }
    }

    List<MatOfPoint> findRedContours(Mat input)
    {
        // A list we'll be using to store the contours we find
        List<MatOfPoint> contoursList = new ArrayList<>();
        List<MatOfPoint> tapeContours = new ArrayList<>();
        List<MatOfPoint> coneContours = new ArrayList<>();

        // Convert the input image to YCrCb color space, then extract the Cr channel
        Imgproc.cvtColor(input, crMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(crMat, crMat, CR_CHAN_IDX);

        // Threshold the Cr channel to form a mask, then run some noise reduction
        Imgproc.threshold(crMat, thresholdRedMat, CR_CHAN_MASK_RED_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        morphMask(thresholdRedMat, morphedRedThreshold);

        // This might not be the best way to do this, but we split the image in two to detect
        // the cones in the upper half, tape in the lower half.
        int greatDivide = findConeTapeVortex(morphedRedThreshold);
        Mat coneHalf = morphedRedThreshold.submat(0, greatDivide, 0, 319);
        Mat tapeHalf = morphedRedThreshold.submat(greatDivide, 239, 0, 319);

        // Ok, now actually look for the contours! We only look for external contours.
        Imgproc.findContours(tapeHalf, tapeContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE,
                new Point(0, greatDivide));
        Imgproc.findContours(coneHalf, coneContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        contoursList.addAll(tapeContours);
        contoursList.addAll(coneContours);
        // We do draw the contours we find, but not to the main input buffer.
        Imgproc.drawContours(contoursOnPlainImageMat, contoursList, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);

        Point leftSide = new Point(0, greatDivide);
        Point rightSide = new Point(319, greatDivide);
        Imgproc.line(contoursOnPlainImageMat, leftSide, rightSide, PURPLE, 2);

        // Lets do our analyzed objects lists here while we have the two lists.
        for (MatOfPoint contour : coneContours) {
            AnalyzeRedConeContour(contour, input);
        }
        for (MatOfPoint contour : tapeContours) {
            AnalyzeRedTapeContour(contour, input);
        }

        coneHalf.release();
        tapeHalf.release();

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
            analyzedCone.aligned = abs(analyzedCone.centralOffset) <= MAX_CONE_OFFSET;
            internalRedConeList.add(analyzedCone);
        }
    }

    boolean findTheRedCone()
    {
        boolean foundCone = false;
        synchronized(lockRedCone) {
            theRedCone.centralOffset = 0;
            theRedCone.corners = new RotatedRect(new double[]{0, 0, 0, 0});
            theRedCone.aligned = false;
            for (AnalyzedCone aCone : internalRedConeList) {
                if (aCone.corners.size.height > theRedCone.corners.size.height) {
                    theRedCone.corners = aCone.corners.clone();
                    theRedCone.centralOffset = aCone.centralOffset;
                    theRedCone.aligned = aCone.aligned;
                    foundCone = true;
                }
            }
        }
        return foundCone;
    }

    void AnalyzeRedTapeContour(MatOfPoint contour, Mat input)
    {
        // Transform the contour to a different format
        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

        // Do a rect fit to the contour, and draw it on the screen
        RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);

        // Make sure it is a cone contour, no need to draw around false pick ups.
        if (isTape(rotatedRectFitToContour))
        {
            AnalyzedTape analyzedTape = new AnalyzedTape();
            analyzedTape.corners = rotatedRectFitToContour;
            analyzedTape.centralOffset = CENTERED_OBJECT.center.x - rotatedRectFitToContour.center.x;
            analyzedTape.aligned = abs(analyzedTape.centralOffset) <= MAX_TAPE_OFFSET;
            // TODO add angle detection
            internalRedTapeList.add(analyzedTape);
        }
    }

    boolean findTheRedTape()
    {
        boolean foundTape = false;
        synchronized(lockRedTape) {
            theRedTape.centralOffset = 0;
            theRedTape.corners = new RotatedRect(new double[]{0, 0, 0, 0});
            theRedTape.aligned = false;
            for (AnalyzedTape aTape : internalRedTapeList) {
                if (aTape.corners.size.height > theRedCone.corners.size.height) {
                    theRedTape.corners = aTape.corners.clone();
                    theRedTape.centralOffset = aTape.centralOffset;
                    theRedTape.aligned = aTape.aligned;
                    theRedTape.angle = aTape.angle;
                    foundTape = true;
                }
            }
        }
        return foundTape;
    }

    public AnalyzedCone getDetectedBlueCone()
    {
        synchronized(lockBlueCone) {
            return new AnalyzedCone(theBlueCone);
        }
    }

    public AnalyzedTape getDetectedBlueTape()
    {
        synchronized(lockBlueTape) {
            return new AnalyzedTape(theBlueTape);
        }
    }

    List<MatOfPoint> findBlueContours(Mat input)
    {
        // A list we'll be using to store the contours we find
        List<MatOfPoint> contoursList = new ArrayList<>();
        List<MatOfPoint> tapeContours = new ArrayList<>();
        List<MatOfPoint> coneContours = new ArrayList<>();

        // Convert the input image to YCrCb color space, then extract the Cb channel
        Imgproc.cvtColor(input, cbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(cbMat, cbMat, CB_CHAN_IDX);

        // Threshold the Cb channel to form a mask, then run some noise reduction
        Imgproc.threshold(cbMat, thresholdBlueMat, CB_CHAN_MASK_BLUE_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        morphMask(thresholdBlueMat, morphedBlueThreshold);

        // This might not be the best way to do this, but we split the image in two to detect
        // the cones in the upper half, tape in the lower half.
        int greatDivide = findConeTapeVortex(morphedBlueThreshold);
        Mat coneHalf = morphedBlueThreshold.submat(0, greatDivide, 0, 319);
        Mat tapeHalf = morphedBlueThreshold.submat(greatDivide, 239, 0, 319);

        // Ok, now actually look for the contours! We only look for external contours.
        Imgproc.findContours(tapeHalf, tapeContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE,
                new Point(0, greatDivide));
        Imgproc.findContours(coneHalf, coneContours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        contoursList.addAll(tapeContours);
        contoursList.addAll(coneContours);
        // We do draw the contours we find, but not to the main input buffer.
        Imgproc.drawContours(contoursOnPlainImageMat, contoursList, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);

        Point leftSide = new Point(0, greatDivide);
        Point rightSide = new Point(319, greatDivide);
        Imgproc.line(contoursOnPlainImageMat, leftSide, rightSide, PURPLE, 2);

        // Lets do our analyzed objects lists here while we have the two lists.
        for (MatOfPoint contour : coneContours) {
            AnalyzeBlueConeContour(contour, input);
        }
        for (MatOfPoint contour : tapeContours) {
            AnalyzeBlueTapeContour(contour, input);
        }

        coneHalf.release();
        tapeHalf.release();

        return contoursList;
    }

    int findConeTapeVortex(Mat tapeConeImage) {
        boolean startedTape = false;
        int startingRow = 239;
        int triggerDelta = 255 * 4;
        int tapeDelta = 255 * 10;
        int vortexRow = 0;
        Scalar lastRowAmplitude;
        Scalar thisRowAmplitude;

        lastRowAmplitude = Core.sumElems(tapeConeImage.row(startingRow));
        for(int row = startingRow;row >= 0;row--) {
            thisRowAmplitude = Core.sumElems(tapeConeImage.row(row));
            if(startedTape == false) {
                if(thisRowAmplitude.val[0] > tapeDelta) {
                    startedTape = true;
                    row -= 20;
                    if(row < 0) {
                        row = 0;
                    }
                    thisRowAmplitude = Core.sumElems(tapeConeImage.row(row));
                    lastRowAmplitude = thisRowAmplitude;
                }
            } else {
                // 10 pixel change in magnitude should be 255 * 10
                if ((thisRowAmplitude.val[0] - lastRowAmplitude.val[0]) >= triggerDelta) {
                    vortexRow = row;
                    break;
                } else {
                    lastRowAmplitude = thisRowAmplitude;
                }
            }
        }
        return vortexRow;
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
            analyzedCone.aligned = abs(analyzedCone.centralOffset) <= MAX_CONE_OFFSET;
            internalBlueConeList.add(analyzedCone);
        }
    }

    boolean findTheBlueCone()
    {
        boolean foundCone = false;
        synchronized(lockBlueCone) {
            theBlueCone.centralOffset = 0;
            theBlueCone.corners = new RotatedRect(new double[]{0, 0, 0, 0});
            theBlueCone.aligned = false;
            for (AnalyzedCone aCone : internalBlueConeList) {
                if (aCone.corners.size.height > theBlueCone.corners.size.height) {
                    theBlueCone.corners = aCone.corners.clone();
                    theBlueCone.centralOffset = aCone.centralOffset;
                    theBlueCone.aligned = aCone.aligned;
                    foundCone = true;
                }
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

    void AnalyzeBlueTapeContour(MatOfPoint contour, Mat input)
    {
        // Transform the contour to a different format
        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

        // Do a rect fit to the contour, and draw it on the screen
        RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);

        // Make sure it is a cone contour, no need to draw around false pick ups.
        if (isTape(rotatedRectFitToContour))
        {
            AnalyzedTape analyzedTape = new AnalyzedTape();
            analyzedTape.corners = rotatedRectFitToContour;
            analyzedTape.centralOffset = CENTERED_OBJECT.center.x - rotatedRectFitToContour.center.x;
            analyzedTape.aligned = abs(analyzedTape.centralOffset) <= MAX_TAPE_OFFSET;
            // TODO add angle detection
            internalBlueTapeList.add(analyzedTape);
        }
    }

    boolean findTheBlueTape()
    {
        boolean foundTape = false;
        synchronized(lockBlueTape) {
            theBlueTape.centralOffset = 0;
            theBlueTape.corners = new RotatedRect(new double[]{0, 0, 0, 0});
            theBlueTape.aligned = false;
            for (AnalyzedTape aTape : internalBlueTapeList) {
                if (aTape.corners.size.height > theBlueCone.corners.size.height) {
                    theBlueTape.corners = aTape.corners.clone();
                    theBlueTape.centralOffset = aTape.centralOffset;
                    theBlueTape.aligned = aTape.aligned;
                    theBlueTape.angle = aTape.angle;
                    foundTape = true;
                }
            }
        }
        return foundTape;
    }

    boolean isTape(RotatedRect rect)
    {
        // We can put whatever logic in here we want to determine the coneness
//        return (rect.size.width > rect.size.height);
        return true;
    }

    public AnalyzedPole getDetectedPole()
    {
        synchronized(lockPole) {
            return new AnalyzedPole(thePole);
        }
    }

    List<MatOfPoint> findYellowContours(Mat input)
    {
        // A list we'll be using to store the contours we find
        List<MatOfPoint> contoursList = new ArrayList<>();

        // Convert the input image to YCrCb color space, then extract the Cb channel
        Imgproc.cvtColor(input, cyMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(cyMat, cyMat, CB_CHAN_IDX);

        // Threshold the Cb channel to form a mask, then run some noise reduction
        Imgproc.threshold(cyMat, thresholdYellowMat, CB_CHAN_MASK_YELLOW_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);
        morphMask(thresholdYellowMat, morphedYellowThreshold);

        // Ok, now actually look for the contours! We only look for external contours.
        Imgproc.findContours(morphedYellowThreshold, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // We do draw the contours we find, but not to the main input buffer.
        Imgproc.drawContours(contoursOnPlainImageMat, contoursList, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);

        return contoursList;
    }

    boolean isPole(RotatedRect rect)
    {
        // We can put whatever logic in here we want to determine the poleness
        // This seems backwards on the camera mounted low.
        return ((rect.size.height > 120) && (rect.size.width > 10) && (rect.size.height > rect.size.width));
//        return true;
    }

    boolean findThePole()
    {
        boolean foundPole = false;
        synchronized (lockPole) {
            thePole.centralOffset = 0;
            thePole.highDistanceOffset = 0;
            thePole.corners = new RotatedRect(new double[]{0, 0, 0, 0, 0});
            thePole.aligned = false;
            thePole.properDistanceHigh = false;
            for (AnalyzedPole aPole : internalPoleList) {
                if (aPole.corners.size.width > thePole.corners.size.width) {
                    thePole.centralOffset = aPole.centralOffset;
                    thePole.highDistanceOffset = aPole.highDistanceOffset;
                    thePole.corners = aPole.corners.clone();
                    thePole.aligned = aPole.aligned;
                    thePole.properDistanceHigh = aPole.properDistanceHigh;
                    foundPole = true;
                }
            }
        }
        return foundPole;
    }

    RotatedRect corRectAngle(RotatedRect rectAngle) {
        RotatedRect corRectAngle = rectAngle.clone();

        while(abs(corRectAngle.angle) > 45.0) {
            if(corRectAngle.angle > 0.0) {
                // Since we shift the angle 90 degrees, height and width flip flop.
                corRectAngle.set(new double[]{corRectAngle.center.x, corRectAngle.center.y,
                    corRectAngle.size.height, corRectAngle.size.width, (corRectAngle.angle - 90.0)});
            } else {
                // Since we shift the angle 90 degrees, height and width flip flop.
                corRectAngle.set(new double[]{corRectAngle.center.x, corRectAngle.center.y,
                        corRectAngle.size.height, corRectAngle.size.width, (corRectAngle.angle + 90.0)});
            }
        }
        return corRectAngle;
    }

    void AnalyzePoleContour(MatOfPoint contour, Mat input)
    {
        // Transform the contour to a different format
        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

        // Do a rect fit to the contour, and draw it on the screen
        RotatedRect rotatedRectFitToContour = corRectAngle(Imgproc.minAreaRect(contour2f));

        // Make sure it is a pole contour, no need to draw around false pick ups.
        if (isPole(rotatedRectFitToContour))
        {
            AnalyzedPole analyzedPole = new AnalyzedPole();
            analyzedPole.corners = rotatedRectFitToContour;
            analyzedPole.centralOffset = CENTERED_OBJECT.center.x - rotatedRectFitToContour.center.x;
            analyzedPole.highDistanceOffset = POLE_HIGH_DISTANCE - rotatedRectFitToContour.size.width;
            analyzedPole.aligned = abs(analyzedPole.centralOffset) <= MAX_POLE_OFFSET;
            analyzedPole.properDistanceHigh = abs(analyzedPole.highDistanceOffset) <= MAX_HIGH_DISTANCE_OFFSET;
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

    static void drawPoleRects(List<AnalyzedPole> rects, Mat drawOn, Scalar color)
    {
        for(AnalyzedPole aPole : rects) {
            drawRotatedRect(aPole.corners, drawOn, color);
        }
    }

    static void drawConeRotatedRects(List<AnalyzedCone> rects, Mat drawOn, Scalar color)
    {
        for(AnalyzedCone aCone : rects) {
            drawRotatedRect(aCone.corners, drawOn, color);
        }
    }

    static void drawTapeRotatedRects(List<AnalyzedTape> rects, Mat drawOn, Scalar color)
    {
        for(AnalyzedTape aTape : rects) {
            drawRotatedRect(aTape.corners, drawOn, color);
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
