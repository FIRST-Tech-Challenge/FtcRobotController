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

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

class CenterstageSuperPipeline implements VisionProcessor {
    protected static boolean leftSide;
    protected static boolean redAlliance;
    public static int spikeMark = 1;
    protected Point sub1PointA = new Point(0, 350);
    protected Point sub1PointB = new Point(60, 410);
    protected Point sub2PointA = new Point(460, 320);
    protected Point sub2PointB = new Point(520, 380);
    protected Point sub3PointA = new Point(580, 100);
    protected Point sub3PointB = new Point(620, 140);
    protected Point spikeMarkCenter = new Point(); // defined dynamically below
//  protected Mat YCrCb = new Mat();
//  protected Mat CrChan = new Mat();
//  protected Mat CbChan = new Mat();
    protected Mat HSV = new Mat();
    protected Mat HueChan = new Mat();
    protected Mat subMat1Hue;
    protected Mat subMat2Hue;
    protected Mat subMat3Hue;
    protected int avg1;
    protected int avg2;
    protected int avg3;
    protected int hueErr1;
    protected int hueErr2;
    protected int hueErr3;
    protected int targetHue;  // target hue number
    protected int maxHueErr;  // maximum allowed hue error

    public CenterstageSuperPipeline(boolean leftSide, boolean redAlliance)
    {
        this.leftSide    = leftSide;
        this.redAlliance = redAlliance;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        // Convert the image from RGB to YCrCb (frame -> YCrCb)
//      Imgproc.cvtColor(frame, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        // Extract the CrCb channels (YCrCb -> CrChan)
//      Core.extractChannel(YCrCb, CrChan, 0);
//      Core.extractChannel(YCrCb, CbChan, 1);
        // NOTE: For RED  we need both large Cr + small Cb
        //       (large Cr with LARGE Cb would be purple, not red)
        //       For BLUE we need both small Cr + large Cb
        //       (small Cr with SMALL Cb would be green, not blue)
        // In poor lighting, red blue & purple are too close together to threshold.

        // Convert the image from RGB to HSV  (frame -> HSV)
        Imgproc.cvtColor(frame, HSV, Imgproc.COLOR_RGB2HSV);
        // Extract the HUE channel data (HSV -> HueChan)
        Core.extractChannel(HSV, HueChan, 0);
        // What target hue  are we looking for on this alliance?
        //   RED team prop      = 160     BLUE team prop      = 120
        //   RED spikemark tape =         BLUE spikemark tape = 
        targetHue = (redAlliance)? 160 : 120;
        maxHueErr = 15;     // how far off can we go and still be the desired color??

        // Define permanent sampling areas from this color channel (HueChan -> submatHue)
        subMat1Hue = HueChan.submat(new Rect(sub1PointA, sub1PointB));
        subMat2Hue = HueChan.submat(new Rect(sub2PointA, sub2PointB));
//      subMat3Hue = HueChan.submat(new Rect(sub3PointA, sub3PointB));

        // Compute the AVERAGE hue number for the sample region
        avg1 = (int)Core.mean(subMat1Hue).val[0];  hueErr1 = Math.abs(avg1-targetHue);
        avg2 = (int)Core.mean(subMat2Hue).val[0];  hueErr2 = Math.abs(avg2-targetHue);
//      avg3 = (int)Core.mean(subMat3Hue).val[0];  hueErr3 = Math.abs(avg3-targetHue);

        // Determine which sample zone was closest to our target hue
        if( (hueErr1 < maxHueErr) && (hueErr1 < hueErr2) ) {
            spikeMark = 1;
        } else if ((hueErr2 < maxHueErr) && (hueErr2 <= hueErr1) ) {
            spikeMark = 2;
        } else {
            spikeMark = 3;
        }

        // Draw 4-pixel-wide RED or BLUE rectangles identifying sample areas on original frame
        if( redAlliance ) {
            Imgproc.rectangle(frame, sub1PointA, sub1PointB, new Scalar(255, 0, 0), 4);
            Imgproc.rectangle(frame, sub2PointA, sub2PointB, new Scalar(255, 0, 0), 4);
            Imgproc.rectangle(frame, sub3PointA, sub3PointB, new Scalar(255, 0, 0), 4);
        } else {
            Imgproc.rectangle(frame, sub1PointA, sub1PointB, new Scalar(0, 0, 255), 4);
            Imgproc.rectangle(frame, sub2PointA, sub2PointB, new Scalar(0, 0, 255), 4);
            Imgproc.rectangle(frame, sub3PointA, sub3PointB, new Scalar(0, 0, 255), 4);
        }

        // Add a purple circle to the middle of the chosen sample
        if( spikeMark == 1 ) {
            spikeMarkCenter.x = (sub1PointA.x + sub1PointB.x) / 2;
            spikeMarkCenter.y = (sub1PointA.y + sub1PointB.y) / 2;
        } else if ( spikeMark == 1 ) {
            spikeMarkCenter.x = (sub2PointA.x + sub2PointB.x) / 2;
            spikeMarkCenter.y = (sub2PointA.y + sub2PointB.y) / 2;
        } else {
            spikeMarkCenter.x = (sub3PointA.x + sub3PointB.x) / 2;
            spikeMarkCenter.y = (sub3PointA.y + sub3PointB.y) / 2;
        }
        Imgproc.circle(frame, spikeMarkCenter, 10, new Scalar(225, 52, 235), -1);

        // Free the allocated submat memory
        subMat1Hue.release();
        subMat1Hue = null;
        subMat2Hue.release();
        subMat2Hue = null;
//      subMat3Hue.release();
//      subMat3Hue = null;

        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
} // CenterstageSuperPipeline
