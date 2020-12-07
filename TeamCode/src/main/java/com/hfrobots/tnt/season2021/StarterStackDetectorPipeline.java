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

import android.telecom.TelecomManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

import lombok.Builder;

@Builder
public class StarterStackDetectorPipeline extends OpenCvPipeline {

    private final Telemetry telemetry;

    private final GripPipelineHulls gripPipeline = new GripPipelineHulls();

    enum WobbleDropZone {
        A, B, C;
    }

    private final Mat displayMat = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        //input.copyTo(displayMat);

        gripPipeline.process(input);

        gripPipeline.cvDilateOutput().copyTo(displayMat);

        ArrayList<MatOfPoint> contours = gripPipeline.convexHullsOutput();

        Imgproc.drawContours(displayMat, contours, -1, new Scalar(255, 30, 30), 2);

        Rect largestRect = null;

        for (MatOfPoint contour : contours) {
            Rect boundingRect = Imgproc.boundingRect(contour);

            if (largestRect == null) {
                largestRect = boundingRect;
                continue;
            }

            if (boundingRect.area() > largestRect.area()) {
                largestRect = boundingRect;
            }
        }

        if (largestRect != null) {
            telemetry.addData("Vision", largestRect.width + " x " + largestRect.height);
            Imgproc.rectangle(displayMat, largestRect.tl(), largestRect.br(), new Scalar(0, 0, 255), 2); // Draw rect
        } else {
            telemetry.addData("Vision", "No rings detected");
        }

        return displayMat;
    }

    private WobbleDropZone whichZone(Mat input) {
        //use alpha build up ranges?
        //outline to height method?
        //average pixel val?

        return WobbleDropZone.A;
    }
}
