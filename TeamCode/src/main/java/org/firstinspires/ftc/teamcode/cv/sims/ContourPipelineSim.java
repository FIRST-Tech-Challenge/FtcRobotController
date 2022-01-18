package org.firstinspires.ftc.teamcode.cv.sims;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

public class ContourPipelineSim extends OpenCvPipeline {
    Scalar GREEN = new Scalar(0, 0, 255);

    // Green                        Y      Cr     Cb    (Do not change Y)
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.50, 0.50);
    public static Scalar scalarUpperYCrCb = new Scalar(150.0, 150.0, 130.0);

    Telemetry telemetry;

    // Green                                             Y      Cr     Cb
    //public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 0.0, 0.0);
    //public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120.0, 120.0);
    // use this picture for you own color https://raw.githubusercontent.com/PinkToTheFuture/OpenCV_FreightFrenzy_2021-2022/main/7e8azlgi.bmp
    // Note that the Cr and Cb values range between 0-255. this means that the origin of the coordinate system is (128,128)

    //Volatile bc accessed by opmode without sync
    public boolean error = false;
    //public Exception debug;

    public double borderLeftX = 0.0;   //fraction of pixels from the left side of the cam to skip
    public double borderRightX = 0.0;   //fraction of pixels from the right of the cam to skip
    public double borderTopY = 0.0;   //fraction of pixels from the top of the cam to skip
    public double borderBottomY = 0.0;   //fraction of pixels from the bottom of the cam to skip

    private int CAMERA_WIDTH;
    private int CAMERA_HEIGHT;

    private int loopCounter = 0;
    private int pLoopCounter = 0;


    // private Mat output = new Mat();

    //private Mat matInit = new Mat();
    //private Mat mat = matInit.submat(new Rect(80.0,80.0,160.0,120.0));

    //private Mat processedInit = new Mat();
    //private Mat processed = processedInit.submat(new Rect(80.0,80.0,160.0,120.0));

    private Mat mat = new Mat();
    private Mat processed = new Mat();
    private Mat output = new Mat();

    private Rect maxRect = new Rect(600,1,1,1);
    private Rect rect = new Rect(600,1,1,1);

    private double maxArea = 0;
    private boolean first = false;

    private final Object sync = new Object();

    public ContourPipelineSim(Telemetry t) {

        telemetry = t;

        this.borderLeftX = 0.25;
        this.borderRightX = 0.25;
        this.borderTopY = 0.25;
        this.borderBottomY = 0.45;

        // Green Range                                      Y      Cr     Cb
        Scalar initScalarLowerYCrCb = new Scalar(0.0, 0.0, 0.0);
        Scalar initScalarUpperYCrCb = new Scalar(150.0, 150.0, 110.0);
        configureScalarLower(initScalarLowerYCrCb.val[0],initScalarLowerYCrCb.val[1],initScalarLowerYCrCb.val[2]);
        configureScalarUpper(initScalarUpperYCrCb.val[0],initScalarUpperYCrCb.val[1],initScalarUpperYCrCb.val[2]);
    }


    public void configureScalarLower(double y, double cr, double cb) {
        scalarLowerYCrCb = new Scalar(y, cr, cb);
    }

    public void configureScalarUpper(double y, double cr, double cb) {
        scalarUpperYCrCb = new Scalar(y, cr, cb);
    }

    public void configureScalarLower(int y, int cr, int cb) {
        scalarLowerYCrCb = new Scalar(y, cr, cb);
    }

    public void configureScalarUpper(int y, int cr, int cb) {
        scalarUpperYCrCb = new Scalar(y, cr, cb);
    }

    @Override
    public Mat processFrame(Mat input) {
        CAMERA_WIDTH = input.width();
        CAMERA_HEIGHT = input.height();
        try {
            // Process Image
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);
            Core.inRange(mat, scalarLowerYCrCb, scalarUpperYCrCb, processed);
            // Core.bitwise_and(input, input, output, processed);

            // Remove Noise
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());
            // GaussianBlur
            Imgproc.GaussianBlur(processed, processed, new Size(5.0, 15.0), 0.00);
            // Find Contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            telemetry.addLine("Drawing countours");
            telemetry.update();

            // Draw Contours, red lines that show color areas that match
            Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0));

            //lock this up to prevent errors when outside threads access the max rect property.
            synchronized (sync) {
                // Loop Through Contours
                for (MatOfPoint contour : contours) {
                    Point[] contourArray = contour.toArray();

                    // Bound Rectangle if Contour is Large Enough
                    if (contourArray.length >= 15) {
                        MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                        Rect rect = Imgproc.boundingRect(areaPoints);

                        // if rectangle is larger than previous cycle or if rectangle is not larger than previous 6 cycles > then replace

                        if (rect.area() > maxArea
                                && rect.x > (borderLeftX * CAMERA_WIDTH) && rect.x + rect.width < CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH)
                                && rect.y > (borderTopY * CAMERA_HEIGHT) && rect.y + rect.height < CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT)
                                || loopCounter - pLoopCounter > 6) {
                            maxArea = rect.area();
                            maxRect = rect;
                            pLoopCounter++;
                            loopCounter = pLoopCounter;
                            first = true;
                        }
                        areaPoints.release();
                    }
                    contour.release();
                }
                if (contours.isEmpty()) {
                    maxRect = new Rect();
                }
            }
            // Draw Rectangles If Area Is At Least 500
            if (maxRect.area() > 1500 && maxRect.area() < 2300){
                Imgproc.rectangle(input, maxRect, new Scalar(0, 255, 0), 2); // GREEN
            }
            // Draw Borders
            Imgproc.rectangle(input, new Rect(
                    (int) (borderLeftX * CAMERA_WIDTH),
                    (int) (borderTopY * CAMERA_HEIGHT),
                    (int) (CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH) - (borderLeftX * CAMERA_HEIGHT)),
                    (int) (CAMERA_HEIGHT - (borderBottomY * CAMERA_WIDTH) - (borderTopY * CAMERA_HEIGHT))
            ), GREEN, 2);

            // Display Data
            Imgproc.putText(input, "Area: " + getRectArea() + " Midpoint: " + getRectMidpointXY().x + " , " + getRectMidpointXY().y, new Point(10, 10), 0, 0.35, new Scalar(255, 255, 255), 1);

            if( getRectMidpointXY().x > 70 &&  getRectMidpointXY().x < 90 ) {
                telemetry.addData("X", "Level 1");
            } else if( getRectMidpointXY().x > 140 &&  getRectMidpointXY().x < 155 ) {
                telemetry.addData("X", "Level 2");
            } else {
                telemetry.addData("X", "Level 3");
            }

            loopCounter++;
        } catch (Exception e) {
            //debug = e;
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
            return maxRect.height;
        }
    }

    public int getRectWidth() {
        synchronized (sync) {
            return maxRect.width;
        }
    }

    public int getRectX() {
        synchronized (sync) {
            return maxRect.x;
        }
    }

    public int getRectY() {
        synchronized (sync) {
            return maxRect.y;
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
            return maxRect.area();
        }
    }
}