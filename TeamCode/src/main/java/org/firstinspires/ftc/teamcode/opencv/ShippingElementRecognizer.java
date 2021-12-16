package org.firstinspires.ftc.teamcode.opencv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ShippingElementRecognizer extends OpenCvPipeline {
    private int shippingHubLevel = 3;
    double leftValue;
    double rightValue;
    public int getShippingHubLevel() {
        return shippingHubLevel;
    }

    public double getLeftValue() {
        return leftValue;
    }

    public double getRightValue() {
        return rightValue;
    }

    // Recognizes the shipping hub level based on where the team shipping element is located
    // Create two possible boxes it can be in
    static final Rect LEFTBOX = new Rect(
            new Point(0, 240),
            new Point(120, 320)
    );
    static final Rect RIGHTBOX = new Rect(
            new Point(121, 240),
            new Point(240, 320)
    );

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Scalar lowHSV = new Scalar(110, 100, 35); // purple lower in hsv
        Scalar highHSV = new Scalar(150, 255, 255); // purple upper in hsv
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV); // convert to hsv
        Core.inRange(input, lowHSV, highHSV, mat); // make purple white, everything else black
        Mat left = mat.submat(LEFTBOX);
        Mat right = mat.submat(RIGHTBOX);
        leftValue = Core.sumElems(left).val[0] / LEFTBOX.area(); // Get white pixel / total pixel count
        rightValue = Core.sumElems(right).val[0] / RIGHTBOX.area(); // Get white pixel / total pixel count
        Imgproc.rectangle(mat, LEFTBOX, new Scalar(255, 0, 0), 2); // draw rectangles around the boxes
        Imgproc.rectangle(mat, RIGHTBOX, new Scalar(255, 0, 0), 2);
        // If neither value is high enough , then the shipping hub level is 3, otherwise we continue
        if (leftValue > 1 || rightValue > 1){
            if (leftValue >= rightValue){
                shippingHubLevel = 1;
                Imgproc.rectangle(mat, LEFTBOX, new Scalar(0, 255, 0), 2);
            } else if (leftValue < rightValue) {
                shippingHubLevel = 2;
                Imgproc.rectangle(mat, RIGHTBOX, new Scalar(0, 255, 0), 2);
            }
        } else {
            shippingHubLevel = 3;
        }

        // I think this stops a memory leak?
        left.release();
        right.release();
        return mat;
    }
}
