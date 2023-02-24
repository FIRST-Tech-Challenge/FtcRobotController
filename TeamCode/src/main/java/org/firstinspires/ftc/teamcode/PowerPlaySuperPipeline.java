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
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;

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
    static class FourPointRect {
        FourPointRect() {
            tl = new Point();
            tr = new Point();
            bl = new Point();
            br = new Point();
            center = new Point();
            topCenter = new Point();
            width = 0.0;
            height = 0.0;
        };

        FourPointRect(RotatedRect fromRect) {
            // NOTE: Y values increase as you go down the screen, so a LOW value of Y
            // means it’s near the top of the screen, and a LARGE value of Y means it’s
            // near the bottom.  The 4 points of the rotated rectangle are numbered
            // counter-clockwise, with point 0 being the largest value of Y (LOWEST
            // point on the screen).
            //     2                        2
            //       3                    1
            //    1           or              3
            //      0                       0
            Point[] points = new Point[4];
            fromRect.points(points);
            if( points[1].y < points[3].y ) {
               // Right-hand example above: top=1&2; bottom=0&3
               tl = (points[1].x <= points[2].x)?  points[1] : points[2];
               tr = (points[1].x <= points[2].x)?  points[2] : points[1];
               bl = (points[0].x <= points[3].x)?  points[0] : points[3];
               br = (points[0].x <= points[3].x)?  points[3] : points[0];
            } else {
               // Left-hand example above: top=2&3; bottom=0&1
               tl = (points[2].x <= points[3].x)?  points[2] : points[3];
               tr = (points[2].x <= points[3].x)?  points[3] : points[2];
               bl = (points[0].x <= points[1].x)?  points[0] : points[1];
               br = (points[0].x <= points[1].x)?  points[1] : points[0];
            }
            // Check for the wonky state
            Point wonkySwap;
            if(tl.y > br.y) {
                wonkySwap = br.clone();
                br = tl.clone();
                tl = wonkySwap.clone();
            }
            if(tr.y > bl.y) {
                wonkySwap = bl.clone();
                bl = tr.clone();
                tr = wonkySwap.clone();
            }
            width  = sqrt(pow((tr.x-tl.x), 2) + pow((tr.y-tl.y), 2));
            height = sqrt(pow((bl.x-tl.x), 2) + pow((bl.y-tl.y), 2));
            center    = fromRect.center.clone();
            topCenter = new Point(new double[] {(tl.x + tr.x)/2.0, (tl.y + tr.y)/2.0});
        }

        FourPointRect(Point tl, Point tr, Point bl, Point br) {
            this.tl = tl.clone();
            this.tr = tr.clone();
            this.bl = bl.clone();
            this.br = br.clone();
            width  = sqrt(pow((tr.x-tl.x), 2) + pow((tr.y-tl.y), 2));
            height = sqrt(pow((bl.x-tl.x), 2) + pow((bl.y-tl.y), 2));
            center    = new Point(new double[]{(tl.x + br.x)/2.0, (tl.y + br.y)/2.0});
            topCenter = new Point(new double[]{(tl.x + tr.x)/2.0, (tl.y + tr.y)/2.0});
        }

        public FourPointRect clone() {
            return new FourPointRect(tl, tr, bl, br);
        }

        public Point tl, tr, bl, br;
        public Point center, topCenter;
        public double width, height;
    };
    // Variables for the signal detection
    ArrayList<Mat> channels = new ArrayList<>(3);
    private Mat rL = new Mat();
    private Mat gL = new Mat();
    private Mat bL = new Mat();

    private Mat rR = new Mat();
    private Mat gR = new Mat();
    private Mat bR = new Mat();

    private Mat allianceRR = new Mat();
    private Mat allianceBR = new Mat();
    private Mat allianceRL = new Mat();
    private Mat allianceBL = new Mat();

    private Mat analyzedPoleImage = new Mat();
    private Mat analyzedBlueConeImage = new Mat();
    private Mat analyzedRedConeImage = new Mat();
    private Mat analyzedBlueTapeImage = new Mat();
    private Mat analyzedRedTapeImage = new Mat();

    private int maxL;
    public int avgRL;
    public int avgGL;
    public int avgBL;

    private int maxR;
    public int avgRR;
    public int avgGR;
    public int avgBR;

    private int allianceMax;
    public int allianceAvgRL;
    public int allianceAvgBL;

    public int allianceAvgRR;
    public int allianceAvgBR;

    private Point markerL = new Point();        // Team Element (populated once we find it!)
    private Point markerR = new Point();        // Team Element (populated once we find it!)

    private Point allianceMarkerL = new Point();
    private Point allianceMarkerR = new Point();

    private Point beaconDetectLeftTl;
    private Point beaconDetectLeftBr;
    private Point beaconDetectRightTl;
    private Point beaconDetectRightBr;
    private Point allianceDetectLeftTl;
    private Point allianceDetectLeftBr;
    private Point allianceDetectRightTl;
    private Point allianceDetectRightBr;

    // Public to be used by opMode
    public int signalZoneL;
    public int signalZoneR;

    public Mat finalAutoImage = new Mat();

    private static String directory;

    public Object lockBlueTape = new Object();
    public Object lockRedTape = new Object();
    public Object lockBlueCone = new Object();
    public Object lockRedCone = new Object();
    public Object lockPole = new Object();
    boolean detectPole, detectRedCone, detectBlueCone, detectSignal, isBlueAlliance, isLeft;
    boolean overrideSide, overrideAlliance, overrideIsLeft, overrideIsBlue;
    String storageFolder = Environment.getExternalStorageDirectory().getPath() + "//FIRST//Webcam//Default";

    public PowerPlaySuperPipeline(boolean signalDetection, boolean poleDetection,
                                  boolean redConeDetection, boolean blueConeDetection,
                                  double center)
    {
        detectSignal = signalDetection;
        detectPole = poleDetection;
        detectRedCone = redConeDetection;
        detectBlueCone = blueConeDetection;

        thePole.analyzedFrame = new Mat();
        theRedCone.analyzedFrame = new Mat();
        theBlueCone.analyzedFrame = new Mat();
        theRedTape.analyzedFrame = new Mat();
        theBlueTape.analyzedFrame = new Mat();

        CENTERED_OBJECT = new FourPointRect(new Point(center - 24, 0.0), new Point(center + 24, 0.0),
                new Point(center - 24, 239.0), new Point(center + 24, 239.0));
        /*
         * We won't turn on signal detection after we start the pipeline.
         */
        beaconDetectLeftTl = new Point(212, 89);  // 20x39 pixel box for signal sleeve
        beaconDetectLeftBr = new Point(231, 127);

        beaconDetectRightTl = new Point(95, 89);  // 20x39 pixel box for signal sleeve
        beaconDetectRightBr = new Point(114, 127);

        allianceDetectLeftTl = new Point(49, 89); // 17x39 pixel box for 5-stack
        allianceDetectLeftBr = new Point(65, 125);

        allianceDetectRightTl = new Point(263, 89); // 17x39 pixel box for 5-stack
        allianceDetectRightBr = new Point(279, 125);
        POLE_HIGH_DISTANCE = polePixelWidthToDistance(POLE_HIGH_PIXEL_WIDTH);
    }

    public void setStorageFolder(String storageDir) {
        storageFolder = storageDir;
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

    // Call these functions to save the image associated with the detected object.
    public void saveConeAutoImage(AnalyzedCone detectedCone) {
        String timeString = new SimpleDateFormat("hh-mm-ss.SSS", Locale.getDefault()).format(new Date());

        createImageStorageFolder( );
        String filePath = directory + "/" + "ConeImage_" + timeString + ".png";
        saveImage(filePath, detectedCone.analyzedFrame);
    }
    public void saveTapeAutoImage(AnalyzedTape detectedTape) {
        String timeString = new SimpleDateFormat("hh-mm-ss.SSS", Locale.getDefault()).format(new Date());

        createImageStorageFolder( );
        String filePath = directory + "/" + "TapeImage_" + timeString + ".png";
        saveImage(filePath, detectedTape.analyzedFrame);
    }
    public void savePoleAutoImage(AnalyzedPole detectedPole) {
        String timeString = new SimpleDateFormat("hh-mm-ss.SSS", Locale.getDefault()).format(new Date());

        createImageStorageFolder( );
        String filePath = directory + "/" + "PoleImage_" + timeString + ".png";
        saveImage(filePath, detectedPole.analyzedFrame);
    }

    protected void saveImage(String filePath, Mat image) {
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                try
                {
                    Imgproc.cvtColor(image, image, Imgproc.COLOR_RGB2BGR);
                    Imgcodecs.imwrite(filePath, image);
                }
                catch (Exception e)
                {
                    e.printStackTrace();
                }
                finally
                {
                    image.release();
                }
            }
        }).start();
    }

    protected void createImageStorageFolder( ) {
        // Create a subdirectory based on DATE
        String dateString = new SimpleDateFormat("yyyy-MM-dd", Locale.getDefault()).format(new Date());
        directory = Environment.getExternalStorageDirectory().getPath() + "//FIRST//Webcam//" + dateString;

        if(overrideAlliance) {
            isBlueAlliance = overrideIsBlue;
        }
        if(overrideSide) {
            isLeft = overrideIsLeft;
        }

        if (isBlueAlliance) {
            if (isLeft) {
                directory += "/blue_left";
            } else {
                directory += "/blue_right";
            }
        } else {
            if (isLeft) {
                directory += "/red_left";
            } else {
                directory += "/red_right";
            }
        }
        // Create the directory structure to store the autonomous image used to start auto.
        File baseDir = new File(directory);
        baseDir.mkdirs();
    }

    public void saveLastAutoImage( ) {
        String timeString = new SimpleDateFormat("hh-mm-ss", Locale.getDefault()).format(new Date());

        createImageStorageFolder( );
        String filePath = directory + "/" + "AutoImage_" + timeString + ".png";
        saveImage(filePath, finalAutoImage);
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
    public DebugObjects debugType = DebugObjects.ConeBlue;

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
    protected FourPointRect CENTERED_OBJECT;

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

    static final double PIXELS_TO_DEGREES = 0.15;
    // This is the allowable distance from the center of the pole to the "center"
    // of the image in degrees with 0.15 degrees per pixel.
    static final double MAX_POLE_OFFSET = 2.7;   // 18 +/- pixels
    // This  is how wide a pole is at the proper scoring distance on a high pole
    static final int POLE_HIGH_PIXEL_WIDTH = 40;
    // Converts the target pixels width to a cm distance of the robot from the pole. The pole is
    // 1" diameter, which is 1.27 cm radius.
    protected double POLE_HIGH_DISTANCE;

    // This is how many pixels wide the pole can vary at the proper scoring distance
    // on a high pole.
    static final int MAX_HIGH_DISTANCE_OFFSET = 3;
    static class AnalyzedPole
    {
        public AnalyzedPole() {
            corners = new FourPointRect();
            alignedCount = 0;
            properDistanceHighCount = 0;
            centralOffset = 0;
            centralOffsetDegrees = 0;
            highDistanceOffset = 0;
            highDistanceOffsetCm = 0;
            aligned = false;
            properDistanceHigh = false;
            analyzedFrame = null;
        }

        public AnalyzedPole clone() {
            AnalyzedPole cloneBaby = new AnalyzedPole();
            cloneBaby.corners = corners.clone();
            cloneBaby.alignedCount = alignedCount;
            cloneBaby.properDistanceHighCount = properDistanceHighCount;
            cloneBaby.centralOffset = centralOffset;
            cloneBaby.centralOffsetDegrees = centralOffsetDegrees;
            cloneBaby.highDistanceOffset = highDistanceOffset;
            cloneBaby.highDistanceOffsetCm = highDistanceOffsetCm;
            cloneBaby.aligned = aligned;
            cloneBaby.properDistanceHigh = properDistanceHigh;

            return cloneBaby;
        }
        FourPointRect corners;
        int alignedCount;
        int properDistanceHighCount;
        double centralOffset;
        double centralOffsetDegrees;
        double highDistanceOffset;
        double highDistanceOffsetCm;
        boolean aligned;
        boolean properDistanceHigh;
        Mat analyzedFrame;
    }

    static final double MAX_CONE_OFFSET = 1.5;  // 10 +/- pixels
    static class AnalyzedCone
    {
        public AnalyzedCone() {
            corners = new FourPointRect();
            alignedCount = 0;
            centralOffset = 0;
            centralOffsetDegrees = 0;
            aligned = false;
            analyzedFrame = null;
        }

        public AnalyzedCone clone() {
            AnalyzedCone cloneBaby = new AnalyzedCone();

            cloneBaby.corners = corners.clone();
            cloneBaby.alignedCount = alignedCount;
            cloneBaby.centralOffset = centralOffset;
            cloneBaby.centralOffsetDegrees = centralOffsetDegrees;
            cloneBaby.aligned = aligned;

            return cloneBaby;
        }

        FourPointRect corners;
        int alignedCount;
        double centralOffset;
        double centralOffsetDegrees;
        boolean aligned;
        Mat analyzedFrame;
    }

    static final double MAX_TAPE_OFFSET = 1.5;
    static class AnalyzedTape
    {
        public AnalyzedTape() {
            corners = new FourPointRect();
            alignedCount = 0;
            centralOffset = 0;
            centralOffsetDegrees = 0;
            aligned = false;
            angle = 0.0;
            analyzedFrame = null;
        };
        public AnalyzedTape clone() {
            AnalyzedTape cloneBaby = new AnalyzedTape();

            cloneBaby.corners = corners.clone();
            cloneBaby.alignedCount = alignedCount;
            cloneBaby.centralOffset = centralOffset;
            cloneBaby.centralOffsetDegrees = centralOffsetDegrees;
            cloneBaby.aligned = aligned;
            cloneBaby.angle = angle;

            return cloneBaby;
        }
        FourPointRect corners;
        int alignedCount;
        double centralOffset;
        double centralOffsetDegrees;
        boolean aligned;
        double angle;
        Mat analyzedFrame;
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

    public void overrideSide(boolean isLeft) {
        overrideSide = true;
        overrideIsLeft = isLeft;
    }

    public void overrideAlliance(boolean isBlue) {
        overrideAlliance = true;
        overrideIsBlue = isBlue;
    }

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
            rL = channels.get(0).submat(new Rect(beaconDetectLeftTl, beaconDetectLeftBr) );
            gL = channels.get(1).submat(new Rect(beaconDetectLeftTl, beaconDetectLeftBr) );
            bL = channels.get(2).submat(new Rect(beaconDetectLeftTl, beaconDetectLeftBr) );

            rR = channels.get(0).submat(new Rect(beaconDetectRightTl, beaconDetectRightBr) );
            gR = channels.get(1).submat(new Rect(beaconDetectRightTl, beaconDetectRightBr) );
            bR = channels.get(2).submat(new Rect(beaconDetectRightTl, beaconDetectRightBr) );

            allianceRL = channels.get(0).submat(new Rect(allianceDetectLeftTl, allianceDetectLeftBr) );
            allianceRR = channels.get(0).submat(new Rect(allianceDetectRightTl, allianceDetectRightBr) );
            allianceBL = channels.get(2).submat(new Rect(allianceDetectLeftTl, allianceDetectLeftBr) );
            allianceBR = channels.get(2).submat(new Rect(allianceDetectRightTl, allianceDetectRightBr) );

            // Average the three sample zones
            avgRL = (int)Core.mean(rL).val[0];
            avgBL = (int)Core.mean(bL).val[0];
            avgGL = (int)Core.mean(gL).val[0];

            avgRR = (int)Core.mean(rR).val[0];
            avgBR = (int)Core.mean(bR).val[0];
            avgGR = (int)Core.mean(gR).val[0];

            allianceAvgRL = (int)Core.mean(allianceRL).val[0];
            allianceAvgRR = (int)Core.mean(allianceRR).val[0];

            allianceAvgBL = (int)Core.mean(allianceBL).val[0];
            allianceAvgBR = (int)Core.mean(allianceBR).val[0];

            // Determine which RBG channel had the highest value
            maxL = Math.max(avgRL, Math.max(avgBL, avgGL));
            maxR = Math.max(avgRR, Math.max(avgBR, avgGR));

            if(overrideSide) {
                if(overrideIsLeft) {
                    allianceMax = Math.max(allianceAvgRL, allianceAvgBL);
                } else {
                    allianceMax = Math.max(allianceAvgRR, allianceAvgBR);
                }
            } else {
                allianceMax = Math.max(allianceAvgRR, Math.max(allianceAvgBR, Math.max(allianceAvgRL, allianceAvgBL)));
            }

            // Draw a circle on the detected team shipping element
            markerL.x = (beaconDetectLeftTl.x + beaconDetectLeftBr.x) / 2;
            markerL.y = (beaconDetectLeftTl.y + beaconDetectLeftBr.y) / 2;

            markerR.x = (beaconDetectRightTl.x + beaconDetectRightBr.x) / 2;
            markerR.y = (beaconDetectRightTl.y + beaconDetectRightBr.y) / 2;

            allianceMarkerL.x = (allianceDetectLeftTl.x + allianceDetectLeftBr.x) / 2;
            allianceMarkerL.y = (allianceDetectLeftTl.y + allianceDetectLeftBr.y) / 2;

            allianceMarkerR.x = (allianceDetectRightTl.x + allianceDetectRightBr.x) / 2;
            allianceMarkerR.y = (allianceDetectRightTl.y + allianceDetectRightBr.y) / 2;

            // Free the allocated submat memory
            rL.release();
            rL = null;
            gL.release();
            gL = null;
            bL.release();
            bL = null;

            rR.release();
            rR = null;
            gR.release();
            gR = null;
            bR.release();
            bR = null;

            allianceRL.release();
            allianceRL = null;
            allianceBL.release();
            allianceBL = null;

            allianceRR.release();
            allianceRR = null;
            allianceBR.release();
            allianceBR = null;

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

            drawConeRects(internalBlueConeList, rectanglesOnPlainImageMat, BLUE);
            drawTapeRects(internalBlueTapeList, rectanglesOnPlainImageMat, BLUE);

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

            drawConeRects(internalRedConeList, rectanglesOnPlainImageMat, BLUE);
            drawTapeRects(internalRedTapeList, rectanglesOnPlainImageMat, BLUE);

            findTheRedCone();
            findTheRedTape();
        }

        // Above here, do not draw on input. All the drawing is done below here so it is not part
        // of the processing.
        if(detectSignal) {
            // Draw rectangles around the sample zone
            Imgproc.rectangle(input, beaconDetectLeftTl, beaconDetectLeftBr, new Scalar(0, 0, 255), 1);
            if(maxL == avgRL) {
                Imgproc.circle(input, markerL, 5, new Scalar(255, 0, 0), -1);
                signalZoneL = 1;
            } else if(maxL == avgGL) {
                Imgproc.circle(input, markerL, 5, new Scalar(0, 255, 0), -1);
                signalZoneL = 2;
            } else if(maxL == avgBL) {
                Imgproc.circle(input, markerL, 5, new Scalar(0, 0, 255), -1);
                signalZoneL = 3;
            } else {
                signalZoneL = 3;
            }

            Imgproc.rectangle(input, beaconDetectRightTl, beaconDetectRightBr, TEAL, 1);
            if(maxR == avgRR) {
                Imgproc.circle(input, markerR, 5, new Scalar(255, 0, 0), -1);
                signalZoneR = 1;
            } else if(maxR == avgGR) {
                Imgproc.circle(input, markerR, 5, new Scalar(0, 255, 0), -1);
                signalZoneR = 2;
            } else if(maxR == avgBR) {
                Imgproc.circle(input, markerR, 5, new Scalar(0, 0, 255), -1);
                signalZoneR = 3;
            } else {
                signalZoneR = 3;
            }

            if(overrideSide) {
                if(overrideIsLeft) {
                    Imgproc.rectangle(input, allianceDetectLeftTl, allianceDetectLeftBr, new Scalar(0, 0, 255), 1);
                    if(allianceMax == allianceAvgRL) {
                        Imgproc.circle(input, allianceMarkerL, 5, new Scalar(0, 0, 255), -1);
                        isBlueAlliance = true;
                        isLeft = true;
                    } else if(allianceMax == allianceAvgBL) {
                        Imgproc.circle(input, allianceMarkerL, 5, new Scalar(255, 0, 0), -1);
                        isBlueAlliance = false;
                        isLeft = true;
                    } else {
                        isBlueAlliance = true;
                        isLeft = true;
                    }
                } else {
                    Imgproc.rectangle(input, allianceDetectRightTl, allianceDetectRightBr, TEAL, 1);
                    if(allianceMax == allianceAvgRR) {
                        Imgproc.circle(input, allianceMarkerR, 5, new Scalar(0, 0, 255), -1);
                        isBlueAlliance = true;
                        isLeft = false;
                    } else if(allianceMax == allianceAvgBR) {
                        Imgproc.circle(input, allianceMarkerR, 5, new Scalar(255, 0, 0), -1);
                        isBlueAlliance = false;
                        isLeft = false;
                    } else {
                        isBlueAlliance = true;
                        isLeft = false;
                    }
                }
            } else {
                Imgproc.rectangle(input, allianceDetectLeftTl, allianceDetectLeftBr, new Scalar(0, 0, 255), 1);
                Imgproc.rectangle(input, allianceDetectRightTl, allianceDetectRightBr, TEAL, 1);
                if(allianceMax == allianceAvgRR) {
                    Imgproc.circle(input, allianceMarkerR, 5, new Scalar(0, 0, 255), -1);
                    isBlueAlliance = true;
                    isLeft = false;
                } else if(allianceMax == allianceAvgBR) {
                    Imgproc.circle(input, allianceMarkerR, 5, new Scalar(255, 0, 0), -1);
                    isBlueAlliance = false;
                    isLeft = false;
                } else if(allianceMax == allianceAvgRL) {
                    Imgproc.circle(input, allianceMarkerL, 5, new Scalar(0, 0, 255), -1);
                    isBlueAlliance = true;
                    isLeft = true;
                } else if(allianceMax == allianceAvgBL) {
                    Imgproc.circle(input, allianceMarkerL, 5, new Scalar(255, 0, 0), -1);
                    isBlueAlliance = false;
                    isLeft = true;
                } else {
                    isBlueAlliance = true;
                    isLeft = true;
                }
            }
            input.copyTo(finalAutoImage);
        }

        if(detectPole) {
            synchronized(lockPole) {
                if (thePole.aligned) {
                    thePole.alignedCount++;
                    drawFourPointRect(thePole.corners, input, GREEN);
                } else {
                    thePole.alignedCount = 0;
                    drawFourPointRect(thePole.corners, input, RED);
                }
                if(thePole.properDistanceHigh) {
                    thePole.properDistanceHighCount++;
                } else {
                    thePole.properDistanceHighCount = 0;
                }
                input.copyTo(analyzedPoleImage);
            }
        }
        if(detectBlueCone) {
            synchronized(lockBlueCone) {
                if (theBlueCone.aligned) {
                    theBlueCone.alignedCount++;
                    drawFourPointRect(theBlueCone.corners, input, GREEN);
                } else {
                    theBlueCone.alignedCount = 0;
                    drawFourPointRect(theBlueCone.corners, input, RED);
                }
                input.copyTo(analyzedBlueConeImage);
            }
            synchronized(lockBlueTape) {
                if (theBlueTape.aligned) {
                    theBlueTape.alignedCount++;
                    drawFourPointRect(theBlueTape.corners, input, GREEN);
                } else {
                    theBlueTape.alignedCount = 0;
                    drawFourPointRect(theBlueTape.corners, input, RED);
                }
                input.copyTo(analyzedBlueTapeImage);
            }
        }
        if(detectRedCone) {
            synchronized(lockRedCone) {
                if (theRedCone.aligned) {
                    theRedCone.alignedCount++;
                    drawFourPointRect(theRedCone.corners, input, GREEN);
                } else {
                    theRedCone.alignedCount = 0;
                    drawFourPointRect(theRedCone.corners, input, RED);
                }
                input.copyTo(analyzedRedConeImage);
            }
            synchronized(lockRedTape) {
                if (theRedTape.aligned) {
                    theRedTape.alignedCount++;
                    drawFourPointRect(theRedTape.corners, input, GREEN);
                } else {
                    theRedTape.alignedCount = 0;
                    drawFourPointRect(theRedTape.corners, input, RED);
                }
                input.copyTo(analyzedRedTapeImage);
            }
        }
        drawFourPointRect(CENTERED_OBJECT, input, BLUE);
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
            AnalyzedCone cloneBaby = theRedCone.clone();
            cloneBaby.analyzedFrame = new Mat();
            analyzedRedConeImage.copyTo(cloneBaby.analyzedFrame);
            return cloneBaby;
        }
    }

    public AnalyzedTape getDetectedRedTape()
    {
        synchronized(lockRedTape) {
            AnalyzedTape cloneBaby = theRedTape.clone();
            cloneBaby.analyzedFrame = new Mat();
            analyzedRedTapeImage.copyTo(cloneBaby.analyzedFrame);
            return cloneBaby;
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
        FourPointRect rectFitToContour = new FourPointRect(Imgproc.minAreaRect(contour2f));

        // Make sure it is a cone contour, no need to draw around false pick ups.
        if (isCone(rectFitToContour))
        {
            AnalyzedCone analyzedCone = new AnalyzedCone();
            analyzedCone.corners = rectFitToContour.clone();
            analyzedCone.centralOffset = CENTERED_OBJECT.center.x - rectFitToContour.center.x;
            analyzedCone.centralOffsetDegrees = analyzedCone.centralOffset * PIXELS_TO_DEGREES;
            analyzedCone.aligned = abs(analyzedCone.centralOffsetDegrees) <= MAX_CONE_OFFSET;
            internalRedConeList.add(analyzedCone);
        }
    }

    boolean findTheRedCone()
    {
        boolean foundCone = false;
        synchronized(lockRedCone) {
            theRedCone.corners = new FourPointRect();
            theRedCone.centralOffset = 0;
            theRedCone.centralOffsetDegrees = 0.0;
            theRedCone.aligned = false;
            for (AnalyzedCone aCone : internalRedConeList) {
                if (aCone.corners.height > theBlueCone.corners.height) {
                    theRedCone.corners = aCone.corners.clone();
                    theRedCone.centralOffset = aCone.centralOffset;
                    theRedCone.centralOffsetDegrees = aCone.centralOffsetDegrees;
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
        FourPointRect rectFitToContour = new FourPointRect(Imgproc.minAreaRect(contour2f));

        // Make sure it is a cone contour, no need to draw around false pick ups.
        if (isTape(rectFitToContour))
        {
            AnalyzedTape analyzedTape = new AnalyzedTape();
            analyzedTape.corners = rectFitToContour.clone();
            analyzedTape.centralOffset = CENTERED_OBJECT.center.x - rectFitToContour.center.x;
            analyzedTape.centralOffsetDegrees = analyzedTape.centralOffset * PIXELS_TO_DEGREES;
            analyzedTape.aligned = abs(analyzedTape.centralOffsetDegrees) <= MAX_TAPE_OFFSET;
            // TODO add angle detection
            internalRedTapeList.add(analyzedTape);
        }
    }

    boolean findTheRedTape()
    {
        boolean foundTape = false;
        synchronized(lockRedTape) {
            theRedTape.corners = new FourPointRect();
            theRedTape.centralOffset = 0;
            theRedTape.centralOffsetDegrees = 0.0;
            theRedTape.aligned = false;
            theRedTape.angle = 0.0;
            for (AnalyzedTape aTape : internalRedTapeList) {
                if (aTape.corners.height > theRedTape.corners.height) {
                    theRedTape.corners = aTape.corners.clone();
                    theRedTape.centralOffset = aTape.centralOffset;
                    theRedTape.centralOffsetDegrees = aTape.centralOffsetDegrees;
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
            AnalyzedCone cloneBaby = theBlueCone.clone();
            cloneBaby.analyzedFrame = new Mat();
            analyzedBlueConeImage.copyTo(cloneBaby.analyzedFrame);
            return cloneBaby;
        }
    }

    public AnalyzedTape getDetectedBlueTape()
    {
        synchronized(lockBlueTape) {
            AnalyzedTape cloneBaby = theBlueTape.clone();
            cloneBaby.analyzedFrame = new Mat();
            analyzedBlueTapeImage.copyTo(cloneBaby.analyzedFrame);
            return cloneBaby;
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
        FourPointRect rectFitToContour = new FourPointRect(Imgproc.minAreaRect(contour2f));

        // Make sure it is a cone contour, no need to draw around false pick ups.
        if (isCone(rectFitToContour))
        {
            AnalyzedCone analyzedCone = new AnalyzedCone();
            analyzedCone.corners = rectFitToContour.clone();
            analyzedCone.centralOffset = CENTERED_OBJECT.center.x - rectFitToContour.center.x;
            analyzedCone.centralOffsetDegrees = analyzedCone.centralOffset * PIXELS_TO_DEGREES;
            analyzedCone.aligned = abs(analyzedCone.centralOffsetDegrees) <= MAX_CONE_OFFSET;
            internalBlueConeList.add(analyzedCone);
        }
    }

    boolean findTheBlueCone()
    {
        boolean foundCone = false;
        synchronized(lockBlueCone) {
            theBlueCone.corners = new FourPointRect();
            theBlueCone.centralOffset = 0;
            theBlueCone.centralOffsetDegrees = 0.0;
            theBlueCone.aligned = false;
            for (AnalyzedCone aCone : internalBlueConeList) {
                if (aCone.corners.height > theBlueCone.corners.height) {
                    theBlueCone.corners = aCone.corners.clone();
                    theBlueCone.centralOffset = aCone.centralOffset;
                    theBlueCone.centralOffsetDegrees = aCone.centralOffsetDegrees;
                    theBlueCone.aligned = aCone.aligned;
                    foundCone = true;
                }
            }
        }
        return foundCone;
    }

    boolean isCone(FourPointRect rect)
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
        FourPointRect rectFitToContour = new FourPointRect(Imgproc.minAreaRect(contour2f));

        // Make sure it is a cone contour, no need to draw around false pick ups.
        if (isTape(rectFitToContour))
        {
            AnalyzedTape analyzedTape = new AnalyzedTape();
            analyzedTape.corners = rectFitToContour.clone();
            analyzedTape.centralOffset = CENTERED_OBJECT.center.x - rectFitToContour.center.x;
            analyzedTape.centralOffsetDegrees = analyzedTape.centralOffset * PIXELS_TO_DEGREES;
            analyzedTape.aligned = abs(analyzedTape.centralOffsetDegrees) <= MAX_TAPE_OFFSET;
            // TODO add angle detection
            internalBlueTapeList.add(analyzedTape);
        }
    }

    boolean findTheBlueTape()
    {
        boolean foundTape = false;
        synchronized(lockBlueTape) {
            theBlueTape.corners = new FourPointRect();
            theBlueTape.centralOffset = 0;
            theBlueTape.centralOffsetDegrees = 0.0;
            theBlueTape.angle = 0.0;
            theBlueTape.aligned = false;
            for (AnalyzedTape aTape : internalBlueTapeList) {
                if (aTape.corners.height > theBlueTape.corners.height) {
                    theBlueTape.corners = aTape.corners.clone();
                    theBlueTape.centralOffset = aTape.centralOffset;
                    theBlueTape.centralOffsetDegrees = aTape.centralOffsetDegrees;
                    theBlueTape.angle = aTape.angle;
                    theBlueTape.aligned = aTape.aligned;
                    foundTape = true;
                }
            }
        }
        return foundTape;
    }

    boolean isTape(FourPointRect rect)
    {
        // We can put whatever logic in here we want to determine the coneness
//        return (rect.size.width > rect.size.height);
        return true;
    }

    public double polePixelWidthToDistance(double pixels) {
        // Pole diameter is 1", so 1.27cm radius.
        return (1.27 / tan(toRadians((pixels / 2) * PIXELS_TO_DEGREES)));
    }

    public AnalyzedPole getDetectedPole()
    {
        synchronized(lockPole) {
            AnalyzedPole cloneBaby = thePole.clone();
            cloneBaby.analyzedFrame = new Mat();
            analyzedPoleImage.copyTo(cloneBaby.analyzedFrame);
            return cloneBaby;
        }
    }

    List<MatOfPoint> findYellowContours(Mat input)
    {
        // A list we'll be using to store the contours we find
        List<MatOfPoint> contoursList = new ArrayList<>();
        Mat heightRestricted = input.submat(0, 120, 0, 319);

        // Convert the input image to YCrCb color space, then extract the Cb channel
        Imgproc.cvtColor(heightRestricted, cyMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(cyMat, cyMat, CB_CHAN_IDX);

        // Threshold the Cb channel to form a mask, then run some noise reduction
        Imgproc.threshold(cyMat, thresholdYellowMat, CB_CHAN_MASK_YELLOW_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);
        morphMask(thresholdYellowMat, morphedYellowThreshold);

        // Ok, now actually look for the contours! We only look for external contours.
        Imgproc.findContours(morphedYellowThreshold, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // We do draw the contours we find, but not to the main input buffer.
        Imgproc.drawContours(contoursOnPlainImageMat, contoursList, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);

        heightRestricted.release();
        return contoursList;
    }

    boolean isPole(FourPointRect rect)
    {
        // We can put whatever logic in here we want to determine the poleness
        // This seems backwards on the camera mounted low.
        return ((rect.height > 80) && (rect.width > 10) && (rect.height > rect.width));
//        return true;
    }

    boolean findThePole()
    {
        boolean foundPole = false;
        synchronized (lockPole) {
            thePole.centralOffset = 0;
            thePole.centralOffsetDegrees = 0;
            thePole.highDistanceOffset = 0;
            thePole.highDistanceOffsetCm = 0;
            thePole.corners = new FourPointRect();
            thePole.aligned = false;
            thePole.properDistanceHigh = false;
            for (AnalyzedPole aPole : internalPoleList) {
                // Find the WIDEST pole
                if (aPole.corners.width > thePole.corners.width) {
                    thePole.centralOffset = aPole.centralOffset;
                    thePole.centralOffsetDegrees = aPole.centralOffsetDegrees;
                    thePole.highDistanceOffset = aPole.highDistanceOffset;
                    thePole.highDistanceOffsetCm = aPole.highDistanceOffsetCm;
                    thePole.corners = aPole.corners.clone();
                    thePole.aligned = aPole.aligned;
                    thePole.properDistanceHigh = aPole.properDistanceHigh;
                    foundPole = true;
                }
            }
        }
        return foundPole;
    }

    void AnalyzePoleContour(MatOfPoint contour, Mat input)
    {
        // Transform the contour to a different format
        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

        // Do a rect fit to the contour, and draw it on the screen
        FourPointRect rectFitToContour = new FourPointRect(Imgproc.minAreaRect(contour2f));

        // Make sure it is a pole contour, no need to draw around false pick ups.
        if (isPole(rectFitToContour))
        {
            AnalyzedPole analyzedPole = new AnalyzedPole();
            analyzedPole.corners = rectFitToContour.clone();
            analyzedPole.centralOffset = CENTERED_OBJECT.center.x - rectFitToContour.center.x;
            analyzedPole.centralOffsetDegrees = analyzedPole.centralOffset * PIXELS_TO_DEGREES;
            analyzedPole.highDistanceOffset = POLE_HIGH_PIXEL_WIDTH - rectFitToContour.width;
            analyzedPole.highDistanceOffsetCm = POLE_HIGH_DISTANCE - polePixelWidthToDistance(rectFitToContour.width);
            analyzedPole.aligned = abs(analyzedPole.centralOffsetDegrees) <= MAX_POLE_OFFSET;
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
            drawFourPointRect(aPole.corners, drawOn, color);
        }
    }

    static void drawConeRects(List<AnalyzedCone> rects, Mat drawOn, Scalar color)
    {
        for(AnalyzedCone aCone : rects) {
            drawFourPointRect(aCone.corners, drawOn, color);
        }
    }

    static void drawTapeRects(List<AnalyzedTape> rects, Mat drawOn, Scalar color)
    {
        for(AnalyzedTape aTape : rects) {
            drawFourPointRect(aTape.corners, drawOn, color);
        }
    }

    static void drawFourPointRect(FourPointRect rect, Mat drawOn, Scalar color)
   {
       /*
        * Draws a rotated rect by drawing each of the 4 lines individually
        */
       Imgproc.line(drawOn, rect.tl, rect.tr, color, 2);
       Imgproc.line(drawOn, rect.tr, rect.br, color, 2);
       Imgproc.line(drawOn, rect.br, rect.bl, color, 2);
       Imgproc.line(drawOn, rect.bl, rect.tl, color, 2);
   }
}
