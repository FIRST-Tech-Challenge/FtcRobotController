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

package com.hfrobots.tnt.season2021;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import lombok.Builder;
import lombok.Setter;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

@Builder
public class StarterStackDetectorPipeline extends OpenCvPipeline {

    private static final String STACK_DETECTOR_TEL_CAPTION = "SDet";
    private final Telemetry telemetry;

    private final GripPipelineHulls gripPipeline = new GripPipelineHulls();

    public enum RingsDetected {
        ZERO, ONE, FOUR, UNKNOWN, SOME;
    }

    private final Mat displayMat = new Mat();

    private final AtomicReference<RingsDetected> ringsDetected = new AtomicReference(RingsDetected.UNKNOWN);

    @Setter
    private volatile boolean startLookingForRings = false;

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(displayMat);

        gripPipeline.process(input);

        ArrayList<MatOfPoint> contours = gripPipeline.convexHullsOutput();

        Imgproc.drawContours(displayMat, contours, -1, new Scalar(255, 30, 30), 2);

        // How do we "rate" the contours as being more likely ring stacks?

        Rect largestBoundingRect = null;

        for (MatOfPoint contour : contours) {
            Rect boundingRect = Imgproc.boundingRect(contour);

            if (largestBoundingRect == null) {
                largestBoundingRect = boundingRect;
            } else if ( boundingRect.area() > largestBoundingRect.area()) {
                largestBoundingRect = boundingRect;
            }
        }

        if (largestBoundingRect != null) {
            Imgproc.rectangle(displayMat, largestBoundingRect.tl(), largestBoundingRect.br(), new Scalar(0, 0, 255), 2); // Draw rect
        }

        if (startLookingForRings) {
            // How do we detect whether this is 0 rings, 1 ring or 4 rings?

            if (largestBoundingRect == null) {
                // we found no rings
                ringsDetected.set(RingsDetected.ZERO);

                telemetry.addData(STACK_DETECTOR_TEL_CAPTION, "-> " + ringsDetected.get().toString());
            } else {
                // Observed heights/widths using VisionTest OpMode
                //
                //          B
                //       D  A  C
                //          E
                //
                // S - Rings have a slant
                // I - Rings are straight
                //
                // W x H in pixels table:
                //
                //      4 Rings     1 Ring      No Rings
                // ----------------------------------------
                // AI   71 x 54     68 x 30         --
                // AS   71 x 50     --              --
                // BI   66 x 43     64 x 25         --
                // BS   68 x 45     --              --
                // CI   72 x 56     68 x 31         --
                // CS   78 x 54     --              --
                // DI   70 x 51     68 x 27         --
                // DS   76 x 52     --              --
                // EI   70 x 56     72 x 31         --
                // ES   80 x 53     --              --

                // FIXME: The following if()s we talked about in our meeting, are height only
                // We know the width, does it help us reject non-starter stacks?

                int height = largestBoundingRect.height;
                int width = largestBoundingRect.width;

                if (height > 20 && height < 35) {
                    ringsDetected.set(RingsDetected.ONE);
                } else if (height > 40 && height < 60) {
                    ringsDetected.set(RingsDetected.FOUR);
                } else {
                    // we think we saw rings, but the height is off
                    // Let auto figure out how it wants to handle that

                    ringsDetected.set(RingsDetected.SOME);
                }

                telemetry.addData("SDet", largestBoundingRect.width + " x " + largestBoundingRect.height + " -> " + ringsDetected.get().toString());

                Log.d(LOG_TAG,
                        "Rings detected: " + largestBoundingRect.width + " x " + largestBoundingRect.height + " -> " + ringsDetected.get().toString());
            }
        }

        return displayMat;
    }

    public RingsDetected getRingsDetected() {
        return ringsDetected.get();
    }
}
