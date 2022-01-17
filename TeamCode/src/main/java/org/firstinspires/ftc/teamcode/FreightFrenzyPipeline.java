package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/* Skystone image procesing pipeline to be run upon receipt of each frame from the camera.
 * Note that the processFrame() method is called serially from the frame worker thread -
 * that is, a new camera frame will not come in while you're still processing a previous one.
 * In other words, the processFrame() method will never be called multiple times simultaneously.
 *
 * However, the rendering of your processed image to the viewport is done in parallel to the
 * frame worker thread. That is, the amount of time it takes to render the image to the
 * viewport does NOT impact the amount of frames per second that your pipeline can process.
 *
 * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
 * frame worker thread. This should not be a problem in the vast majority of cases. However,
 * if you're doing something weird where you do need it synchronized with your OpMode thread,
 * then you will need to account for that accordingly.
 */
class FreightFrenzyPipeline extends OpenCvPipeline
{
    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */
    private Mat YCrCb = new Mat();
    private Mat Cb    = new Mat();
    private Mat Cr    = new Mat();
    private Mat subMat1;
    private Mat subMat2;
    private Mat subMat3;
    private int max;
    private int avg1;
    private int avg2;
    private int avg3;
    private Point marker = new Point();        // Team Element (populated once we find it!)
    private Point sub1PointA;
    private Point sub1PointB;
    private Point sub2PointA;
    private Point sub2PointB;
    private Point sub3PointA;
    private Point sub3PointB;

    // Points for alignment
    private Mat alignBlueMat1;
    private Mat alignBlueMat2;
    private Mat alignBlueMat3;
    private int alignBlueAvg1;
    private int alignBlueAvg2;
    private int alignBlueAvg3;
    private Mat alignRedMat1;
    private Mat alignRedMat2;
    private Mat alignRedMat3;
    private int alignRedAvg1;
    private int alignRedAvg2;
    private int alignRedAvg3;

    private Point alignment1RedPointA = new Point(47,212);
    private Point alignment1RedPointB = new Point(57,222);
    private Point alignment2RedPointA = new Point(160,212);
    private Point alignment2RedPointB = new Point(170,222);
    private Point alignment3RedPointA = new Point(273,212);
    private Point alignment3RedPointB = new Point(285,222);
    private final static double colorRedThreshold = 140.0;

    private Point alignment1BluePointA = new Point(40,212);
    private Point alignment1BluePointB = new Point(50,222);
    private Point alignment2BluePointA = new Point(158,212);
    private Point alignment2BluePointB = new Point(168,222);
    private Point alignment3BluePointA = new Point(277,212);
    private Point alignment3BluePointB = new Point(287,222);
    private final static double colorBlueThreshold = 140.0;

    // Public statics to be used by opMode
    public static int blockLevel;

    public static int leftBlueAverage;
    public static boolean alignedBlueLeft;
    public static int centerBlueAverage;
    public static boolean alignedBlueCenter;
    public static int rightBlueAverage;
    public static boolean alignedBlueRight;

    public static int leftRedAverage;
    public static boolean alignedRedLeft;
    public static int centerRedAverage;
    public static boolean alignedRedCenter;
    public static int rightRedAverage;
    public static boolean alignedRedRight;

    private final boolean redAlliance;
    private final boolean duckySide;

    FreightFrenzyPipeline(boolean redAlliance, boolean duckySide) {
        this.redAlliance = redAlliance;
        this.duckySide = duckySide;
        if(redAlliance) {
            if(duckySide) {
                sub1PointA = new Point( 37,190); // 15x15 pixels on LEFT
                sub1PointB = new Point( 52,205);
                sub2PointA = new Point(156,190); // 15x15 pixels on CENTER
                sub2PointB = new Point(171,205);
                sub3PointA = new Point(277,190); // 15x15 pixels on RIGHT
                sub3PointB = new Point(292,205);
            } else {
                sub1PointA = new Point( 37,190); // 15x15 pixels on LEFT
                sub1PointB = new Point( 52,205);
                sub2PointA = new Point(156,190); // 15x15 pixels on CENTER
                sub2PointB = new Point(171,205);
                sub3PointA = new Point(265,190); // 15x15 pixels on RIGHT (limited by barrier!)
                sub3PointB = new Point(280,205);
            }
        } else {
            if(duckySide) {
                sub1PointA = new Point( 30,190); // 15x15 pixels on LEFT
                sub1PointB = new Point( 45,205);
                sub2PointA = new Point(155,190); // 15x15 pixels on CENTER
                sub2PointB = new Point(170,205);
                sub3PointA = new Point(272,190); // 15x15 pixels on RIGHT
                sub3PointB = new Point(287,205);

            } else {
                sub1PointA = new Point( 50,190); // 15x15 pixels on LEFT  (limited by barrier!)
                sub1PointB = new Point( 65,205);
                sub2PointA = new Point(155,190); // 15x15 pixels on CENTER
                sub2PointB = new Point(170,205);
                sub3PointA = new Point(272,190); // 15x15 pixels on RIGHT
                sub3PointB = new Point(287,205);
            }
        }
    }

    @Override
    public Mat processFrame(Mat input)
    {
        // Convert image frame from RGB to YCrCb
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        // Extract the Cb channel from the image frame
        Core.extractChannel(YCrCb, Cb, 2);
        // Extract the Cr channel from the image frame
        Core.extractChannel(YCrCb, Cr, 3);
        // Pull data for the three sample zones from the Cb channel
        subMat1 = Cb.submat(new Rect(sub1PointA,sub1PointB) );
        subMat2 = Cb.submat(new Rect(sub2PointA,sub2PointB) );
        subMat3 = Cb.submat(new Rect(sub3PointA,sub3PointB) );
        alignBlueMat1 = Cb.submat(new Rect(alignment1BluePointA, alignment1BluePointB));
        alignBlueMat2 = Cb.submat(new Rect(alignment2BluePointA, alignment2BluePointB));
        alignBlueMat3 = Cb.submat(new Rect(alignment3BluePointA, alignment3BluePointB));
        alignRedMat1 = Cr.submat(new Rect(alignment1RedPointA, alignment1RedPointB));
        alignRedMat2 = Cr.submat(new Rect(alignment2RedPointA, alignment2RedPointB));
        alignRedMat3 = Cr.submat(new Rect(alignment3RedPointA, alignment3RedPointB));

        // Average the three sample zones
        avg1 = (int)Core.mean(subMat1).val[0];
        avg2 = (int)Core.mean(subMat2).val[0];
        avg3 = (int)Core.mean(subMat3).val[0];
        alignBlueAvg1 = (int)Core.mean(alignBlueMat1).val[0];
        alignBlueAvg2 = (int)Core.mean(alignBlueMat2).val[0];
        alignBlueAvg3 = (int)Core.mean(alignBlueMat3).val[0];
        alignRedAvg1 = (int)Core.mean(alignRedMat1).val[0];
        alignRedAvg2 = (int)Core.mean(alignRedMat2).val[0];
        alignRedAvg3 = (int)Core.mean(alignRedMat3).val[0];

        // Draw alignment rectangles
        Imgproc.rectangle(input, alignment1BluePointA, alignment1BluePointB, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, alignment2BluePointA, alignment2BluePointB, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, alignment3BluePointA, alignment3BluePointB, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, alignment1RedPointA, alignment1RedPointB, new Scalar(255, 0, 0), 1);
        Imgproc.rectangle(input, alignment2RedPointA, alignment2RedPointB, new Scalar(255, 0, 0), 1);
        Imgproc.rectangle(input, alignment3RedPointA, alignment3RedPointB, new Scalar(255, 0, 0), 1);

        // Draw rectangles around the sample zones
        Imgproc.rectangle(input, sub1PointA, sub1PointB, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, sub2PointA, sub2PointB, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, sub3PointA, sub3PointB, new Scalar(0, 0, 255), 1);
        // Determine which sample zone had the lowest contrast from blue (lightest color)
        max = Math.min(avg1, Math.min(avg2, avg3));
        // Draw a circle on the detected team shipping element
        if(max == avg1) {
            marker.x = (sub1PointA.x + sub1PointB.x) / 2;
            marker.y = (sub1PointA.y + sub1PointB.y) / 2;
            Imgproc.circle(input, marker, 5, new Scalar(225, 52, 235), -1);
            blockLevel = 1;
        } else if(max == avg2) {
            marker.x = (sub2PointA.x + sub2PointB.x) / 2;
            marker.y = (sub2PointA.y + sub2PointB.y) / 2;
            Imgproc.circle(input, marker, 5, new Scalar(225, 52, 235), -1);
            blockLevel = 2;
        } else if(max == avg3) {
            marker.x = (sub3PointA.x + sub3PointB.x) / 2;
            marker.y = (sub3PointA.y + sub3PointB.y) / 2;
            Imgproc.circle(input, marker, 5, new Scalar(225, 52, 235), -1);
            blockLevel = 3;
        } else {
            blockLevel = 3;
        }

        leftBlueAverage = alignBlueAvg1;
        alignedBlueLeft = (alignBlueAvg1 >= colorBlueThreshold);
        centerBlueAverage = alignBlueAvg2;
        alignedBlueCenter = (alignBlueAvg2 >= colorBlueThreshold);
        rightBlueAverage = alignBlueAvg3;
        alignedBlueRight = (alignBlueAvg3 >= colorBlueThreshold);

        leftRedAverage = alignRedAvg1;
        alignedRedLeft = (alignRedAvg1 >= colorRedThreshold);
        centerRedAverage = alignRedAvg2;
        alignedRedCenter = (alignRedAvg2 >= colorRedThreshold);
        rightRedAverage = alignRedAvg3;
        alignedRedRight = (alignRedAvg3 >= colorRedThreshold);

        // Free the allocated submat memory
        subMat1.release();
        subMat1 = null;
        subMat2.release();
        subMat2 = null;
        subMat3.release();
        subMat3 = null;
        alignBlueMat1.release();
        alignBlueMat1 = null;
        alignBlueMat2.release();
        alignBlueMat2 = null;
        alignBlueMat3.release();
        alignBlueMat3 = null;
        alignRedMat1.release();
        alignRedMat1 = null;
        alignRedMat2.release();
        alignRedMat2 = null;
        alignRedMat3.release();
        alignRedMat3 = null;

        return input;
    }
} // FreightFrenzyPipeline

