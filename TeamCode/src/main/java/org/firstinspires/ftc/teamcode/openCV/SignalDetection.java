package org.firstinspires.ftc.teamcode.openCV;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SignalDetection extends OpenCvPipeline {

    Telemetry telemetry = null;

    // Initialize values
    String cameraName;
    Mat mat = new Mat();

    static final Rect LEFT_BOX_WAREHOUSE = new Rect(
            new Point(50, 120),
            new Point(150, 220)
    );
    static final Rect RIGHT_BOX_WAREHOUSE = new Rect(
            new Point(190, 120),
            new Point(290, 220)
    );
    static final Rect LEFT_BOX_SPINNER = new Rect(
            new Point(0, 140),
            new Point(100, 230)
    );
    static final Rect RIGHT_BOX_SPINNER = new Rect(
            new Point(200, 140),
            new Point(300, 230)
    );

    // Threshold for when element is considered visible
    static double PERCENT_COLOR_THRESHOLD = 0.1;

    static Rect LEFT_RECTANGLE;

    static Rect RIGHT_RECTANGLE;

    // Constructor
    public SignalDetection(String cn, Telemetry t) {
        telemetry = t;

        //  Update local variables
        cameraName = cn;

        LEFT_RECTANGLE = LEFT_BOX_SPINNER;
        RIGHT_RECTANGLE = RIGHT_BOX_SPINNER;
    }

    @Override
    public Mat processFrame(Mat input) {
        /* Create a monochrome image with orange areas white and non-orange areas black
        An area is considered orange when it's HSV lies between the lower and upper HSV threshold */
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);

        // Orange
//        Scalar lowHSV = new Scalar(5, 100, 100); //Lower HSV
//        Scalar highHSV = new Scalar(30, 255, 255); //Upper HSV

        // Purple
//        Scalar lowHSV = new Scalar(125,100,100);
//        Scalar highHSV = new Scalar(160,255,255);

        // Green
//        Scalar lowHSV = new Scalar(50,100,100);
//        Scalar highHSV = new Scalar(100,255,255);

        // Orange YCrCb
        Scalar lowHSV = new Scalar(0, 140, 0);
        Scalar highHSV = new Scalar(255, 200, 100);

        // Purple YCrCb
//        Scalar lowHSV = new Scalar(0, 140, 128);
//        Scalar highHSV = new Scalar(255, 180, 255);

        // Green YCrCb
//        Scalar lowHSV = new Scalar(0, 0, 0);
//        Scalar highHSV = new Scalar(255, 110, 128);

        Core.inRange(mat, lowHSV, highHSV, mat); //Update mat to show black and white areas

        // Create two submats to read data from
        Mat left = mat.submat(LEFT_RECTANGLE);
        Mat right = mat.submat(RIGHT_RECTANGLE);

        // Calculate percent of orange in submats
        double leftValue = Core.sumElems(left).val[0] / LEFT_RECTANGLE.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_RECTANGLE.area() / 255;

        left.release();
        right.release();

        // Compare percentages to the thresholds
        boolean elementLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean elementRight = rightValue > PERCENT_COLOR_THRESHOLD;

        // Convert mat back to RGB
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        // Draw rectangles around submats
        Scalar colorElement = new Scalar(0,255,0);
        Scalar colorEmpty = new Scalar(255,0,0);

        Scalar colorLeft = elementLeft ? colorElement : colorEmpty;
        Scalar colorRight = elementRight ? colorElement : colorEmpty;

        // Imgproc.rectangle(mat, LEFT_RECTANGLE, (location == Start.CustomElement.LEFT && cameraName == "Webcam 2") || (location == Start.CustomElement.LEFT && cameraName == "Webcam 1") ? colorElement : colorEmpty);
        // Imgproc.rectangle(mat, RIGHT_RECTANGLE, (location == Start.CustomElement.MID && cameraName == "Webcam 1") || (location == Start.CustomElement.MID && cameraName == "Webcam 2") ? colorElement : colorEmpty);

        Imgproc.rectangle(mat, LEFT_RECTANGLE, colorLeft);
        Imgproc.rectangle(mat, RIGHT_RECTANGLE, colorRight);

        return mat;
    }
}












//Jaron was here