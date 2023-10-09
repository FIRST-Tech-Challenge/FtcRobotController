package org.firstinspires.ftc.teamcode.src.v2.Vision;

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

// Credits to team 7303 RoboAvatars, adjusted by team 3954 Pink to the Future

/**
 * A {@link OpenCvPipeline} that is used to detect color between two ranges.
 */
public class PipeLine extends OpenCvPipeline {
    // Pink, the default color                         Y      Cr     Cb    (Do not change Y)
//    public static Scalar scalarLowerYCrCb = new Scalar(0, 255, 200);
//    public static Scalar scalarUpperYCrCb = new Scalar(255, 140, 255);
    public Scalar Green = new Scalar(60, 255, 255);
    //PINK RGB 196, 23, 112


    // Yellow, freight or ducks!
//     public static Scalar scalarLowerYellow = new Scalar(0.0, 100.0, 0.0);
//    public static Scalar scalarUpperYellow = new Scalar(255.0, 170.0, 120.0);

    // Green                                             Y      Cr     Cb
    public Scalar scalarLowerYCrCb = new Scalar(0, 0, 0);
    public Scalar scalarUpperYCrCb = new Scalar(255, 120, 120);

    // Use this picture for you own color https://github.com/PinkToTheFuture/OpenCV_FreightFrenzy_2021-2022/blob/main/YCbCr.jpeg
    // Note that the Cr and Cb values range between 0-255. this means that the origin of the coordinate system is (128,128)
    private final Mat mat = new Mat();
    private final Mat processed = new Mat();
    private final Object sync = new Object();
    // Volatile because accessed by OpMode without sync
    public volatile boolean error = false;
    public volatile Exception debug;
    private double borderLeftX;     //fraction of pixels from the left side of the cam to skip
    private double borderRightX;    //fraction of pixels from the right of the cam to skip
    private double borderTopY;      //fraction of pixels from the top of the cam to skip
    private double borderBottomY;   //fraction of pixels from the bottom of the cam to skip
    private int CAMERA_WIDTH;
    private int CAMERA_HEIGHT;
    private int loopCounter = 0;
    private int pLoopCounter = 0;
    /**
     * maxRect is the bounding box surrounding the largest contiguous blob of detected color within the image frame
     */
    private Rect maxRect = new Rect(600, 1, 1, 1);
    private double maxArea = 0;
    private boolean first = false;

    /**
     * Constructs object and sets the area within the camera frame to use
     *
     * @param borderLeftX   The left border position (pixels)
     * @param borderRightX  The Right border position (pixels)
     * @param borderTopY    The top border position (pixels)
     * @param borderBottomY The bottom border position (pixels)
     */
    public PipeLine(double borderLeftX, double borderRightX, double borderTopY, double borderBottomY) {
        this.borderLeftX = borderLeftX;
        this.borderRightX = borderRightX;
        this.borderTopY = borderTopY;
        this.borderBottomY = borderBottomY;
    }

    /**
     * Sets the lower bound for color detection
     *
     * @param y  Y value in the y-cr-cb color space
     * @param cr cr value in the y-cr-cb color space
     * @param cb cb value in the y-cr-cb color space
     */
    public void configureScalarLower(double y, double cr, double cb) {
        scalarLowerYCrCb = new Scalar(y, cr, cb);
    }
    //public void configureScalarLowerYellow(double y, double cr, double cb){
    //scalarLowerYellow = new Scalar(y,cr,cb);
//    }

    /**
     * Sets the upper bound for color detection
     *
     * @param y  Y value in the y-cr-cb color space
     * @param cr cr value in the y-cr-cb color space
     * @param cb cb value in the y-cr-cb color space
     */
    public void configureScalarUpper(double y, double cr, double cb) {
        scalarUpperYCrCb = new Scalar(y, cr, cb);
    }
    // public void configureScalarUpperYellow(double y, double cr, double cb){
    //scalarUpperYellow = new Scalar(y,cr,cb);
//    }

    /**
     * Sets the lower bound for color detection
     *
     * @param y  Y value in the y-cr-cb color space
     * @param cr cr value in the y-cr-cb color space
     * @param cb cb value in the y-cr-cb color space
     */
    public void configureScalarLower(int y, int cr, int cb) {
        scalarLowerYCrCb = new Scalar(y, cr, cb);
    }

    /**
     * Sets the upper bound for color detection
     *
     * @param y  Y value in the y-cr-cb color space
     * @param cr cr value in the y-cr-cb color space
     * @param cb cb value in the y-cr-cb color space
     */
    //public void configureScalarUpper(int y, int cr, int cb) {
    //scalarUpperYCrCb = new Scalar(y, cr, cb);
    //}

    /**
     * Sets the area within the camera frame to use
     *
     * @param borderLeftX   The left border position (pixels)
     * @param borderRightX  The Right border position (pixels)
     * @param borderTopY    The top border position (pixels)
     * @param borderBottomY The bottom border position (pixels)
     */
    public void configureBorders(double borderLeftX, double borderRightX, double borderTopY, double borderBottomY) {
        this.borderLeftX = borderLeftX;
        this.borderRightX = borderRightX;
        this.borderTopY = borderTopY;
        this.borderBottomY = borderBottomY;
    }

    /**
     * Processes a image frame
     *
     * @param input The image frame to process
     * @return A processed image frame
     */
    @Override
    public Mat processFrame(Mat input) {
        CAMERA_WIDTH = input.width();
        CAMERA_HEIGHT = input.height();
        try {
            // Process Image
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);
            Core.inRange(mat, scalarLowerYCrCb, scalarUpperYCrCb, processed);
            // Remove Noise
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());
            // GaussianBlur
            Imgproc.GaussianBlur(processed, processed, new Size(11, 11), 0.00);
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

                        if (rect.area() > maxArea
                                && rect.x + (rect.width / 2.0) > (borderLeftX * CAMERA_WIDTH)
                                && rect.x + (rect.width / 2.0) < CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH)
                                && rect.y + (rect.height / 2.0) > (borderTopY * CAMERA_HEIGHT)
                                && rect.y + (rect.height / 2.0) < CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT)

                                || loopCounter - pLoopCounter > 6
                                && rect.x + (rect.width / 2.0) > (borderLeftX * CAMERA_WIDTH)
                                && rect.x + (rect.width / 2.0) < CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH)
                                && rect.y + (rect.height / 2.0) > (borderTopY * CAMERA_HEIGHT)
                                && rect.y + (rect.height / 2.0) < CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT)
                        ) {
                            maxArea = rect.area();
                            maxRect = rect;
                            pLoopCounter++;
                            loopCounter = pLoopCounter;
                            first = true;
                        } else if (loopCounter - pLoopCounter > 10) {
                            maxArea = new Rect().area();
                            maxRect = new Rect();
                        }

                        areaPoints.release();
                    }
                    contour.release();
                }
                if (contours.isEmpty()) {
                    maxRect = new Rect(600, 1, 1, 1);
                }
            }
            // Draw Rectangles If Area Is At Least 500
            if (first && maxRect.area() > 500) {
                Imgproc.rectangle(input, maxRect, new Scalar(0, 255, 0), 2);
            }
            // Draw Borders
            Imgproc.rectangle(input, new Rect(
                    (int) (borderLeftX * CAMERA_WIDTH),
                    (int) (borderTopY * CAMERA_HEIGHT),
                    (int) (CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH) - (borderLeftX * CAMERA_WIDTH)),
                    (int) (CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT) - (borderTopY * CAMERA_HEIGHT))
            ), Green, 2);

            // Display Data
            Imgproc.putText(input, "Area: " + getRectArea() + " Midpoint: " + getRectMidpointXY().x + " , " + getRectMidpointXY().y, new Point(5, CAMERA_HEIGHT - 5), 0, 0.6, new Scalar(255, 255, 255), 2);

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

    /**
     * Gets the height of {@link PipeLine#maxRect}
     *
     * @return the height of {@link PipeLine#maxRect}
     */
    public int getRectHeight() {
        synchronized (sync) {
            return maxRect.height;
        }
    }

    /**
     * Gets the width of {@link PipeLine#maxRect}
     *
     * @return the width of {@link PipeLine#maxRect}
     */
    public int getRectWidth() {
        synchronized (sync) {
            return maxRect.width;
        }
    }

    /**
     * Gets the x position of {@link PipeLine#maxRect}
     *
     * @return the x position of {@link PipeLine#maxRect}
     */
    public int getRectX() {
        synchronized (sync) {
            return maxRect.x;
        }
    }

    /**
     * Gets the y position of {@link PipeLine#maxRect}
     *
     * @return the y position of {@link PipeLine#maxRect}
     */
    public int getRectY() {
        synchronized (sync) {
            return maxRect.y;
        }
    }

    /**
     * Gets the center x position of {@link PipeLine#maxRect}
     *
     * @return the center x position of {@link PipeLine#maxRect}
     */
    public double getRectMidpointX() {
        synchronized (sync) {
            return getRectX() + (getRectWidth() / 2.0);
        }
    }

    /**
     * Gets the center y position of {@link PipeLine#maxRect}
     *
     * @return the center y position of {@link PipeLine#maxRect}
     */
    public double getRectMidpointY() {
        synchronized (sync) {
            return getRectY() + (getRectHeight() / 2.0);
        }
    }

    /**
     * Gets the center point of {@link PipeLine#maxRect}
     *
     * @return the center point of {@link PipeLine#maxRect}
     */
    public Point getRectMidpointXY() {
        synchronized (sync) {
            return new Point(getRectMidpointX(), getRectMidpointY());
        }
    }

    /**
     * Gets the aspect ratio of {@link PipeLine#maxRect}
     *
     * @return the aspect ratio of {@link PipeLine#maxRect}
     */
    public double getAspectRatio() {
        synchronized (sync) {
            return getRectArea() / (CAMERA_HEIGHT * CAMERA_WIDTH);
        }
    }

    /**
     * Gets the area of {@link PipeLine#maxRect}
     *
     * @return the area of {@link PipeLine#maxRect}
     */
    public double getRectArea() {
        synchronized (sync) {
            return maxRect.area();
        }
    }
}