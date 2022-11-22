package org.firstinspires.ftc.teamcode.openCV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

// Credits to team 7303 RoboAvatars, adjusted by team 3954 Pink to the Future, further adjusted by team 18859 Stanislas Tech Team

public class DuckPipeline extends OpenCvPipeline {
    Scalar HOT_PINK = new Scalar(196, 23, 112);

    // Pink, the default color                         Y      Cr     Cb    (Do not change Y)
    // public static Scalar scalarLowerYCrCb = new Scalar(0.0, 150.0, 120.0);
    // public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    // Yellow, freight or ducks!
    // public static Scalar scalarLowerYCrCb = new Scalar(10.0, 150.0, 0.0);
    // public static Scalar scalarUpperYCrCb = new Scalar(255.0, 100.0, 138.0);
    public static Scalar lowHSV = new Scalar(23, 50, 70);
    public static Scalar highHSV = new Scalar(32, 255, 255);

    // Green                                             Y      Cr     Cb
    // public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 0.0, 0.0);
    // public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120.0, 120.0);

    // Use this picture for you own color https://github.com/PinkToTheFuture/OpenCV_FreightFrenzy_2021-2022/blob/main/YCbCr.jpeg
    // Note that the Cr and Cb values range between 0-255. this means that the origin of the coordinate system is (128,128)

    // Volatile because accessed by OpMode without sync
    public volatile boolean error = false;
    public volatile Exception debug;

    private int CAMERA_WIDTH;
    private int CAMERA_HEIGHT;
    private double PERCENT_COLOR_THRESHOLD = 0.4;

    private int loopCounter = 0;
    private int pLoopCounter = 0;

    private final Mat mat = new Mat();
    private final Mat processed = new Mat();

    // private Rect maxRect = new Rect(300,1,1,1);
    private Rect duckRect = new Rect(300,1,1,1);

    private double maxArea = 0;
    private boolean first = false;

    private final Object sync = new Object();

    public DuckPipeline() {

    }

    @Override
    public Mat processFrame(Mat input) {
        CAMERA_WIDTH = input.width();
        CAMERA_HEIGHT = input.height();
        try {
            // Process Image
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(mat, lowHSV, highHSV, processed);
            // Remove Noise
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());
            // GaussianBlur
            Imgproc.GaussianBlur(processed, processed, new Size(5.0, 15.0), 0.00);
            // Find Contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            // Draw Contours
            Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0));

            // Lock this up to prevent errors when outside threads access the max rect property.
            synchronized (sync) {
                // Loop Through Contours
                for (MatOfPoint contour : contours) {
                    Point[] contourArray = contour.toArray();

                    // Bound Rectangle if Contour is Large Enough
                    if (contourArray.length >= 15) {
                        MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                        Rect rect = Imgproc.boundingRect(areaPoints);
                        Mat rectSubmat = processed.submat(rect);
                        double rectValue = Core.sumElems(rectSubmat).val[0] / rect.area() / 255;

                        if (rectValue > PERCENT_COLOR_THRESHOLD) {
                            duckRect = rect;
                            Imgproc.putText(input, "(" + getRectMidpointX() + "," + getRectMidpointY() + ")", new Point(0, 300), 0, 0.6, new Scalar(255, 255, 255), 2);
                            Imgproc.rectangle(input, duckRect, new Scalar(255,0,0), 2);
                            // Imgproc.putText(input, "(" + duckRect + ")", new Point(duckRect.x, duckRect.y), 0, 0.6, new Scalar(255, 255, 255), 2);
                            break;
                        }

                        areaPoints.release();
                    }
                    contour.release();
                }

            }


            loopCounter++;
        } catch (Exception e) {
            debug = e;
            error = true;
        }
        return input;
    }
    /*
    Synchronize these operations as the user code could be incorrect otherwise, i.e a property is read
    while the same rectangle is being processed in the pipeline, leading to some values being not
    synced.
     */

    public int getRectHeight() {
        synchronized (sync) {
            return duckRect.height;
        }
    }
    public int getRectWidth() {
        synchronized (sync) {
            return duckRect.width;
        }
    }
    public int getRectX() {
        synchronized (sync) {
            return duckRect.x;
        }
    }
    public int getRectY() {
        synchronized (sync) {
            return duckRect.y;
        }
    }
    public double getRectMidpointX() {
        synchronized (sync) {
            return getRectX() + (getRectWidth() / 2.0);
        }
    }
    public double getRectMidpointY() {
        synchronized (sync) {
            return getRectY() + (getRectHeight() / 2.0);
        }
    }
    public Point getRectMidpointXY() {
        synchronized (sync) {
            return new Point(getRectMidpointX(), getRectMidpointY());
        }
    }
    public double getAspectRatio() {
        synchronized (sync) {
            return getRectArea() / (CAMERA_HEIGHT * CAMERA_WIDTH);
        }
    }
    public double getRectArea() {
        synchronized (sync) {
            return duckRect.area();
        }
    }
}