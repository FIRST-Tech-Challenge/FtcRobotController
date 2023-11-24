package org.firstinspires.ftc.teamcode.subsystems;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.util.Utilities;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
@Config
public class ColorDetectionPipeline extends OpenCvPipeline {
    public static double X_OFFSET = 60;
    public static double Y_OFFSET  = 20;

    static final int STREAM_WIDTH = 1280; // resolution of camera
    static final int STREAM_HEIGHT = 960; // resolution of camera

    Mat zoomedInput = new Mat();
    Mat HLS = new Mat();
    public int avgH, avgL, avgS;

    // To zoom in (x2)
    Rect viewScope = new Rect(new Point(STREAM_WIDTH/4, STREAM_HEIGHT/4), new Point(STREAM_WIDTH * 3/4, STREAM_HEIGHT * 3/4));

    static final int WidthRectA = 180;
    static final int HeightRectA = 100;

    static final Point RectATopLeftAnchor = new Point(((STREAM_WIDTH/2 - WidthRectA) / 2), ((STREAM_HEIGHT/2 - HeightRectA) / 2));

    Point RectATLCorner = new Point(RectATopLeftAnchor.x - X_OFFSET, RectATopLeftAnchor.y - Y_OFFSET);
    Point RectABRCorner = new Point(RectATopLeftAnchor.x + WidthRectA -X_OFFSET, RectATopLeftAnchor.y + HeightRectA - Y_OFFSET);

    boolean stopped = false;

    public ColorDetectionPipeline()
    {
        Log.v("vision", "ColorDetectionPipeline called.");
    }

    /*
     * This function takes the RGB frame, converts to HLS, and splits individual channels
     */
    ArrayList<Mat> inputMatToHLS(Mat input) {

        Imgproc.cvtColor(input, HLS, Imgproc.COLOR_RGB2HLS);
        ArrayList<Mat> HLSChannels = new ArrayList<Mat>(3);
        Core.split(HLS, HLSChannels);
        return HLSChannels;
    }


    @Override
    public void init(Mat firstFrame) {

        Log.v("vision", "init called.");
    }

    @Override
    public Mat processFrame(Mat input) {
        ;
        if (stopped) {
            return input;
        }

        Log.v("vision", "processFrame called.");
        zoomedInput = input.submat(viewScope);

        Log.v("vision", String.format("Creating submat at (%4.2f, %4.2f), (%4.2f, %4.2f)", RectATLCorner.x, RectATLCorner.y, RectABRCorner.x, RectABRCorner.y));
        Mat interetedArea = zoomedInput.submat(new Rect(RectATLCorner, RectABRCorner));
        Log.v("vision", "submat created.");

        ArrayList<Mat> matInHLS = inputMatToHLS(interetedArea);
        avgH = (int) Core.mean(matInHLS.get(0)).val[0];
        avgL = (int) Core.mean(matInHLS.get(1)).val[0];
        avgS = (int) Core.mean(matInHLS.get(2)).val[0];
        interetedArea.release(); // don't leak memory!
        matInHLS.get(0).release(); // don't leak memory!
        matInHLS.get(1).release(); // don't leak memory!
        matInHLS.get(2).release(); // don't leak memory!

        Imgproc.rectangle( // rings
                zoomedInput, // Buffer to draw on
                RectATLCorner, // First point which defines the rectangle
                RectABRCorner, // Second point which defines the rectangle
                new Scalar(0,0,255), // The color the rectangle is drawn in
                5); // Thickness of the rectangle lines

        Log.v("vision", String.format("processFrame result: avgH = %d, avgL = %d, avgS = %d.", avgH, avgL, avgS));

        return zoomedInput;
    }

    public int getConeOrientation() {
        /*
         * Use YCrCb
         * 1: Green  - Y: 68  Cr: 111 Cb: 125 (with flashlight: 198, 92, 122)
         * 2: White  - Y: 135 Cr: 123 Cb: 136 (wiht flashlight: 253, 126, 128)
         * 3: Purple - Y: 78  Cr: 129 Cb: 144 (with flashlight:  194, 122, 161)
         *
         * Use HLS
         * 1: Green  - H: 76  L: 61  S: 74  (with flashlight: 85, 138, 70)
         * 2: White  - H: 112 L: 142 S: 27  (with flashlight: 60-80, 254, 255)
         * 3: Purple - H: 127 L: 83  S: 56  (with flashlight: 115-126, 221, 233)
         * 4: Red    - H: 167 L: 68  S: 163 (with flashlight: 172, 134, 154)
         * Look up color from https://colorizer.org
         * H (0-360)        H x 2       Green 60-180 => 30-90, Purple 250 - 340 => 125-320, Red 320 - 20 => 160-10
         * S (0-100, %)     S / 256
         * L (0-100, %)     L / 256
         */

        Utilities.getSharedUtility().telemetry.addData("Sleeve Color:", avgH);

        //algorithm for finding the closest color
        /*if (avgH > 50 && avgH < 80) {
            return 1;  // purple (1st pos) 118
        } else if (avgH > 14 && avgH < 17) {
            return 2;  // orange (2nd pos) 16
        } else if (avgH > 115 || avgH < 120) {
            return 3;  // green (3rd pos) 78
        } else {
            return 1; // a guess...
        }*/
        int purple1 = 124;
        int orange2 = 15;
        int green3 = 68;
        int distFrom1 = Math.abs(avgH-purple1);
        int distFrom2 = Math.abs(avgH-orange2);
        int distFrom3 = Math.abs(avgH-green3);
        if (distFrom1<distFrom2 && distFrom1<distFrom3){
            Log.v("vision", "detected: " + 1);
            return 1;
        }
        else if (distFrom2<=distFrom1 && distFrom2<=distFrom3){
            Log.v("vision", "detected: " + 2);
            return 2;
        }
        else if (distFrom3<=distFrom1 && distFrom3<distFrom2){
            Log.v("vision", "detected: " + 3);
            return 3;
        }
        else{
            Log.v("vision", "Nothing detected, Default is 3");
            return 3;
        }
    }

    public void stop() {
        stopped = true;
    }
}