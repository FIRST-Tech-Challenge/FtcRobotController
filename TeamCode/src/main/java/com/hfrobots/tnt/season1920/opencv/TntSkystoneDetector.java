/*
 Copyright (c) 2020 HF Robotics (http://www.hfrobots.com)
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
*/

package com.hfrobots.tnt.season1920.opencv;

import android.util.Log;

import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.GrayscaleFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.disnodeteam.dogecv.scoring.MaxAreaScorer;
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer;
import com.disnodeteam.dogecv.scoring.RatioScorer;
import com.hfrobots.tnt.corelib.util.Logging;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.io.FileOutputStream;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.time.Instant;
import java.time.OffsetDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.TimeZone;
import java.util.concurrent.atomic.AtomicReference;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class TntSkystoneDetector extends DogeCVDetector {
    public static final String INNER_ZONE_NAME = "Inner";
    public static final String MIDDLE_ZONE_NAME = "Middle";
    public  static final String OUTER_ZONE_NAME = "Outer";

    public DogeCV.AreaScoringMethod areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Setting to decide to use MaxAreaScorer or PerfectAreaScorer

    //Create the default filters and scorers
    public DogeCVColorFilter blackFilter = new GrayscaleFilter(0, 25);
    public DogeCVColorFilter yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 70); //Default Yellow blackFilter

    public RatioScorer ratioScorer = new RatioScorer(1.25, 3); // Used to find the short face of the stone
    public MaxAreaScorer maxAreaScorer = new MaxAreaScorer(0.01);                    // Used to find largest objects
    public PerfectAreaScorer perfectAreaScorer = new PerfectAreaScorer(5000, 0.05); // Used to find objects near a tuned area value

    // Results of the detector
    private Point screenPosition = new Point();
    private Rect foundRect = new Rect(); // Found rect

    private Mat rawImage = new Mat();
    private Mat workingMat = new Mat();
    private Mat displayMat = new Mat();
    private Mat blackMask = new Mat();
    private Mat yellowMask = new Mat();
    private Mat hierarchy = new Mat();

    private Telemetry telemetry;

    private DetectionZone innerZone = new DetectionZone(INNER_ZONE_NAME, 0, 106);
    private DetectionZone middleZone = new DetectionZone(MIDDLE_ZONE_NAME, 106, 212);
    private DetectionZone outerZone = new DetectionZone(OUTER_ZONE_NAME, 212, 320);

    private AtomicReference<DetectionZone> bestScoringZone = new AtomicReference<>(middleZone); // have to start somewhere!

    private AtomicReference<DetectionZone> secondBestScoringZone = new AtomicReference<>(null);

    List<DetectionZone> detectionZones = new ArrayList<>();

    private boolean startSearching;

    private boolean stopPipeline;

    private OutputStream logStream;

    private PrintWriter logWriter;

    private SimpleDateFormat formatter = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSSZ");

    public void setTelemetry(final Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public TntSkystoneDetector() {
        detectorName = "TNT Skystone Detector";
        detectionZones.add(innerZone);
        detectionZones.add(middleZone);
        detectionZones.add(outerZone);

        try {
            String timestamp = formatter.format(new Date());

            logStream = new FileOutputStream(Logging.getLogFile("skystone-detector-" + timestamp + ".log"));
            logWriter = new PrintWriter(logStream);
        } catch (Throwable t) {
            Log.e(LOG_TAG, "Unable to open detector logfile", t);
        }
    }

    @Override
    public Mat process(Mat input) {

        if (stopPipeline) {
            return input;
        }

        input.copyTo(rawImage);
        input.copyTo(workingMat);
        input.copyTo(displayMat);
        input.copyTo(blackMask);

        yellowFilter.process(workingMat.clone(), yellowMask);

        List<MatOfPoint> contoursYellow = new ArrayList<>();

        Imgproc.findContours(yellowMask, contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat, contoursYellow, -1, new Scalar(255, 30, 30), 2);


        // Current result
        Rect bestRect = foundRect;
        double bestDifference = Double.MAX_VALUE; // MAX_VALUE since less difference = better

        // Loop through the contours and score them, searching for the best result
        for (MatOfPoint cont : contoursYellow) {
            double score = calculateScore(cont); // Get the difference score using the scoring API

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(cont);
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0, 0, 255), 2); // Draw rect

            // If the result is better then the previously tracked one, set this rect as the new best
            if (score < bestDifference) {
                bestDifference = score;
                bestRect = rect;
            }
        }

        Imgproc.rectangle(blackMask, bestRect.tl(), bestRect.br(), new Scalar(255, 255, 255), 1, Imgproc.LINE_4, 0);
        blackFilter.process(workingMat.clone(), blackMask);
        List<MatOfPoint> contoursBlack = new ArrayList<>();

        Imgproc.findContours(blackMask, contoursBlack, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat, contoursBlack, -1, new Scalar(40, 40, 40), 2);

        for (MatOfPoint cont : contoursBlack) {
            double score = calculateScore(cont); // Get the difference score using the scoring API

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(cont);
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0, 0, 255), 2); // Draw rect

            // Check if we should start scoring/detection
            // once autonomous has started.
            if (startSearching) {
                innerZone.accumulateBlackContourArea(cont, rect);
                middleZone.accumulateBlackContourArea(cont, rect);
                outerZone.accumulateBlackContourArea(cont, rect);
            }


            // If the result is better then the previously tracked one, set this rect as the new best
            if (score < bestDifference) {
                bestDifference = score;
                bestRect = rect;
                Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(255, 165, 0), 2); // Draw rect

            } else {
                Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0, 0, 255), 2); // Draw rect

            }
        }

        if (bestRect != null) {
            screenPosition = new Point(bestRect.x, bestRect.y);
            foundRect = bestRect;
            found = true;
        } else {
            found = false;
        }

        Collections.sort(detectionZones);
        DetectionZone skystoneZone = detectionZones.get(detectionZones.size() - 1);
        bestScoringZone.set(skystoneZone);
        secondBestScoringZone.set(detectionZones.get(1));

        // Show where the pipeline is looking for skystone contours
        Imgproc.rectangle(displayMat, new Point(0, 0), new Point(106, 239), new Scalar(255, 192, 203), 4);
        Imgproc.rectangle(displayMat, new Point(106, 0), new Point(212, 239), new Scalar(255, 192, 203), 4);
        Imgproc.rectangle(displayMat, new Point(212, 0), new Point(320, 239), new Scalar(255, 192, 203), 4);

        if (startSearching) {
            String timestamp = formatter.format(new Date());

            if (telemetry != null) {
                telemetry.addData("BLA", innerZone.getTotalBlackArea() + ", "
                        + middleZone.getTotalBlackArea() + ", "
                        + outerZone.getTotalBlackArea());
            }

            if (logWriter != null) {
                writeLogLine(timestamp + " black area: " + innerZone.getTotalBlackArea() + ", "
                            + middleZone.getTotalBlackArea() + ", "
                            + outerZone.getTotalBlackArea());
            }

            if (telemetry != null) {
                telemetry.addData("DZ", "skystone in %s?", skystoneZone.getZoneName());
            }

            if (logWriter != null) {
                writeLogLine(String.format(timestamp + " skystone in %s?",
                        skystoneZone.getZoneName()));
            }

            Log.d(LOG_TAG, "skystone in " + skystoneZone.getZoneName() + ", "
                    + skystoneZone.getTotalBlackArea());
        }

        switch (stageToRenderToViewport) {
            case THRESHOLD: {
                Imgproc.cvtColor(blackMask, blackMask, Imgproc.COLOR_GRAY2BGR);

                return blackMask;
            }
            case RAW_IMAGE: {
                return rawImage;
            }
            default: {
                return displayMat;
            }
        }
    }

    @Override
    public void useDefaults() {
        addScorer(ratioScorer);

        // Add scorers depending on the selected mode
        if (areaScoringMethod == DogeCV.AreaScoringMethod.MAX_AREA) {
            addScorer(maxAreaScorer);
        }

        if (areaScoringMethod == DogeCV.AreaScoringMethod.PERFECT_AREA) {
            addScorer(perfectAreaScorer);
        }
    }

    public void startSearching() {
        startSearching = true;
    }

    public void stopPipeline() {
        stopPipeline = true;

        if (logWriter != null) {
            try {
                logWriter.flush();
                logWriter.close();
            } catch (Throwable t) {
                Log.e(LOG_TAG, "Unable to close log writer", t);
            }
        }

        if (logStream != null) {
            try {
                logStream.flush();
                logStream.close();
            } catch (Throwable t) {
                Log.e(LOG_TAG, "Unable to close log stream", t);
            }
        }
    }

    public DetectionZone getBestScoringZone() {
        return bestScoringZone.get();
    }

    public DetectionZone getSecondBestScoringZone() {
        return secondBestScoringZone.get();
    }

    private void writeLogLine(String line) {
        try {
            logWriter.println(line);
        } catch (Throwable t) {
            Log.e(LOG_TAG, "unable to write to vision log", t);
        }
    }
}
