package org.firstinspires.ftc.teamcode.opencv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ShippingElementRecognizer extends OpenCvPipeline {
    private int shippingHubLevel = 0;
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

    //Recognizes the shipping hub level based on where the team shipping element is located
    static final Rect LEFTBOX = new Rect(
            new Point(0, 200),
            new Point(120, 280)
    );
    static final Rect RIGHTBOX = new Rect(
            new Point(121, 200),
            new Point(240, 280)
    );

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Scalar lowHSV = new Scalar(120, 100, 35); // duck lower
        Scalar highHSV = new Scalar(140, 255, 255); // duck upper
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
        Core.inRange(input, lowHSV, highHSV, mat);
        Mat left = mat.submat(LEFTBOX);
        Mat right = mat.submat(RIGHTBOX);
        leftValue = Core.sumElems(left).val[0] / LEFTBOX.area();
        rightValue = Core.sumElems(right).val[0] / RIGHTBOX.area();
        Imgproc.rectangle(mat, LEFTBOX, new Scalar(255, 0, 0), 2);
        Imgproc.rectangle(mat, RIGHTBOX, new Scalar(255, 0, 0), 2);
        if (leftValue > 15 || rightValue > 15){
            if (leftValue <= rightValue){
                shippingHubLevel = 1;
                Imgproc.rectangle(mat, LEFTBOX, new Scalar(0, 0, 255), 2);
            } else if (leftValue > rightValue) {
                shippingHubLevel = 2;
                Imgproc.rectangle(mat, RIGHTBOX, new Scalar(0, 0, 255), 2);
            }
        } else {
            shippingHubLevel = 3;
        }
        return mat;
    }
}
