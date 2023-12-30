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
import org.opencv.imgcodecs.Imgcodecs;
import android.os.Environment;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

class CenterstageSuperPipeline implements VisionProcessor {
    protected static boolean leftSide;
    protected static boolean redAlliance;
    public static int spikeMark = 1;
    protected String storageFolder = Environment.getExternalStorageDirectory().getPath() + "//FIRST//Webcam//Default";
    protected boolean saveNextSpikeMark = false;
    // NOTE: webcam resolution for the row/col values below is 1280 x 800
    // LEFT ALLIANCE (autonomous alignment uses RIGHT side of the field tile, away from truss)
    protected Point sub1PointALeft = new Point(330, 360); //Point1 is left spike mark, Point2 is center, Point3 is Right
    protected Point sub1PointBLeft = new Point(460, 500);
    protected Point sub2PointALeft = new Point(730, 370);
    protected Point sub2PointBLeft = new Point(840, 485);
    protected Point sub3PointALeft = new Point(1130, 370);
    protected Point sub3PointBLeft = new Point(1275, 510);
    // RIGHT ALLIANCE (autonomous alignment uses LEFT side of the field tile, away from truss)
    protected Point sub1PointARight = new Point(80, 390);
    protected Point sub1PointBRight = new Point(170, 480);
    protected Point sub2PointARight = new Point(560, 375);
    protected Point sub2PointBRight = new Point(650, 455);
    protected Point sub3PointARight = new Point(930, 395);
    protected Point sub3PointBRight = new Point(1020, 485);
    
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

        //================ YCrCb-based detection ================
        // Convert the image from RGB to YCrCb (frame -> YCrCb)
//      Imgproc.cvtColor(frame, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        // Extract the CrCb channels (YCrCb -> CrChan)
//      Core.extractChannel(YCrCb, CrChan, 0);  // Cr (red)
//      Core.extractChannel(YCrCb, CbChan, 1);  // Cb (blue)
        // NOTE: For RED  we need both large Cr + small Cb
        //       (large Cr with LARGE Cb would be purple, not red)
        //       For BLUE we need both small Cr + large Cb
        //       (small Cr with SMALL Cb would be green, not blue)
        // In poor lighting, red blue & purple are too close together to threshold.
        //================ YCrCb based detection ================

        //================ HSV-based detection ================
        // Convert the image from RGB to HSV  (frame -> HSV)
        Imgproc.cvtColor(frame, HSV, Imgproc.COLOR_RGB2HSV);
        // Extract the HUE channel data (HSV -> HueChan)
        Core.extractChannel(HSV, HueChan, 0);
        // What target hue  are we looking for on this alliance?
        //   RED team prop  = 175     BLUE team prop =
        //   RED lion head  =         BLUE lion head = 105
        targetHue = (redAlliance)? 175 : 105;
        maxHueErr = 25;     // how far off can we go and still be the desired color??

        // Define 3 permanent sampling areas from this color channel (HueChan -> submatHue)
        if( leftSide ) {
            subMat1Hue = HueChan.submat(new Rect(sub1PointALeft, sub1PointBLeft));
            subMat2Hue = HueChan.submat(new Rect(sub2PointALeft, sub2PointBLeft));
            subMat3Hue = HueChan.submat(new Rect(sub3PointALeft, sub3PointBLeft));
        } else {
            subMat1Hue = HueChan.submat(new Rect(sub1PointARight, sub1PointBRight));
            subMat2Hue = HueChan.submat(new Rect(sub2PointARight, sub2PointBRight));
            subMat3Hue = HueChan.submat(new Rect(sub3PointARight, sub3PointBRight));
        }
        // Compute the AVERAGE hue number for the sample region
        avg1 = (int)Core.mean(subMat1Hue).val[0];  hueErr1 = Math.abs(avg1-targetHue);
        avg2 = (int)Core.mean(subMat2Hue).val[0];  hueErr2 = Math.abs(avg2-targetHue);
        avg3 = (int)Core.mean(subMat3Hue).val[0];  hueErr3 = Math.abs(avg3-targetHue);

        // Determine which sample zone was closest to our target hue (and within max allowed error)
        if( (hueErr1 < maxHueErr) && (hueErr1 < hueErr2) && (hueErr1 < hueErr3) ) {
            spikeMark = 1;
        } else if ((hueErr2 < maxHueErr) && (hueErr2 <= hueErr1) && (hueErr2 <= hueErr3) ) {
            spikeMark = 2;
        } else if((hueErr3 < maxHueErr) && (hueErr3 <= hueErr1) && (hueErr3 <= hueErr2)) {
            spikeMark = 3;
        }
        else { //default to spike mark away from the truss
            spikeMark = (leftSide)? 1:3;
            // NOTE: We don't use CENTER as our default, as that's where the team prop is
            // located during setup -- meaning we wouldn't know if auto-detection wasn't
            // working as it would default to the CORRECT LOCATION.  If we see spike marks
            // 1 or 3 during setup (when it should be 2) then we know we have an issue.
        }

        // Draw 4-pixel-wide RED or BLUE rectangles identifying sample areas on original frame
        if( redAlliance ) {
            if(leftSide) {
                Imgproc.rectangle(frame, sub1PointALeft, sub1PointBLeft, new Scalar(255, 0, 0), 4);
                Imgproc.rectangle(frame, sub2PointALeft, sub2PointBLeft, new Scalar(255, 0, 0), 4);
                Imgproc.rectangle(frame, sub3PointALeft, sub3PointBLeft, new Scalar(255, 0, 0), 4);
            } else {
                Imgproc.rectangle(frame, sub1PointARight, sub1PointBRight, new Scalar(255, 0, 0), 4);
                Imgproc.rectangle(frame, sub2PointARight, sub2PointBRight, new Scalar(255, 0, 0), 4);
                Imgproc.rectangle(frame, sub3PointARight, sub3PointBRight, new Scalar(255, 0, 0), 4);
            }
        } else {
            if(leftSide) {
                Imgproc.rectangle(frame, sub1PointALeft, sub1PointBLeft, new Scalar(0, 0, 255), 4);
                Imgproc.rectangle(frame, sub2PointALeft, sub2PointBLeft, new Scalar(0, 0, 255), 4);
                Imgproc.rectangle(frame, sub3PointALeft, sub3PointBLeft, new Scalar(0, 0, 255), 4);
            } else {
                Imgproc.rectangle(frame, sub1PointARight, sub1PointBRight, new Scalar(0, 0, 255), 4);
                Imgproc.rectangle(frame, sub2PointARight, sub2PointBRight, new Scalar(0, 0, 255), 4);
                Imgproc.rectangle(frame, sub3PointARight, sub3PointBRight, new Scalar(0, 0, 255), 4);
            }
        }

        // Add a purple circle to the middle of the chosen sample
        if( spikeMark == 1 ) {
            if(leftSide) {
                spikeMarkCenter.x = (sub1PointALeft.x + sub1PointBLeft.x) / 2;
                spikeMarkCenter.y = (sub1PointALeft.y + sub1PointBLeft.y) / 2;
            } else {
                spikeMarkCenter.x = (sub1PointARight.x + sub1PointBRight.x) / 2;
                spikeMarkCenter.y = (sub1PointARight.y + sub1PointBRight.y) / 2;
            }
        } else if ( spikeMark == 2 ) {
            if(leftSide) {
                spikeMarkCenter.x = (sub2PointALeft.x + sub2PointBLeft.x) / 2;
                spikeMarkCenter.y = (sub2PointALeft.y + sub2PointBLeft.y) / 2;
            } else {
                spikeMarkCenter.x = (sub2PointARight.x + sub2PointBRight.x) / 2;
                spikeMarkCenter.y = (sub2PointARight.y + sub2PointBRight.y) / 2;
            }
        } else { // spikeMark == 3
            if(leftSide) {
                spikeMarkCenter.x = (sub3PointALeft.x + sub3PointBLeft.x) / 2;
                spikeMarkCenter.y = (sub3PointALeft.y + sub3PointBLeft.y) / 2;
            } else {
                spikeMarkCenter.x = (sub3PointARight.x + sub3PointBRight.x) / 2;
                spikeMarkCenter.y = (sub3PointARight.y + sub3PointBRight.y) / 2;
            }
        }
        Imgproc.circle(frame, spikeMarkCenter, 10, new Scalar(225, 52, 235), -1);

        // Free the allocated submat memory
        subMat1Hue.release();
        subMat1Hue = null;
        subMat2Hue.release();
        subMat2Hue = null;
        subMat3Hue.release();
        subMat3Hue = null;

        if(saveNextSpikeMark)
        {
            saveSpikeMarkAutoImage(frame);
            saveNextSpikeMark = false;
        }
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
    public void saveSpikeMarkAutoImage() { saveNextSpikeMark = true; }
    public void setStorageFolder(String storageDir) {
        storageFolder = storageDir;
    }
    protected void saveSpikeMarkAutoImage(Mat detectedSpikeMark ) {
        String timeString = new SimpleDateFormat("hh-mm-ss", Locale.getDefault()).format(new Date());

        String filePath = storageFolder + "/" + "AutoImage_" + timeString + ".png";
        saveImage(filePath, detectedSpikeMark);
    }
    protected void saveImage(String filePath, Mat image) {
        Mat cloneBaby = image.clone();
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                try
                {
                    Imgproc.cvtColor(cloneBaby, cloneBaby, Imgproc.COLOR_RGB2BGR);
                    Imgcodecs.imwrite(filePath, cloneBaby);
                }
                catch (Exception e)
                {
                    e.printStackTrace();
                }
                finally
                {
                    cloneBaby.release();
                }
            }
        }).start();
    }
} // CenterstageSuperPipeline
