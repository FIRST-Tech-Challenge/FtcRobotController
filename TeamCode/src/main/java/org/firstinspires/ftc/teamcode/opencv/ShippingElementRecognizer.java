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

    public int getShippingHubLevel() {
        return shippingHubLevel;
    }

    //Recognizes the shipping hub level based on where the team shipping element is located
    static final Rect LEFTBOX = new Rect(
            new Point(0, 40),
            new Point(106, 140)
    );
    static final Rect CENTERBOX = new Rect(
            new Point(106, 40),
            new Point(212, 140)
    );
    static final Rect RIGHTBOX = new Rect(
            new Point(212, 40),
            new Point(320, 140)
    );

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Scalar lowHSV = new Scalar(130, 51, 0); // duck lower
        Scalar highHSV = new Scalar(160, 255, 255); // duck upper
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
        Core.inRange(input, lowHSV, highHSV, mat);
        Mat left = mat.submat(LEFTBOX);
        Mat center = mat.submat(CENTERBOX);
        Mat right = mat.submat(RIGHTBOX);
        double leftValue = Core.sumElems(left).val[0] / LEFTBOX.area();
        double centerValue = Core.sumElems(center).val[0] / CENTERBOX.area();
        double rightValue = Core.sumElems(right).val[0] / RIGHTBOX.area();
        Imgproc.rectangle(mat, LEFTBOX, new Scalar(255, 0, 0), 2);
        Imgproc.rectangle(mat, CENTERBOX, new Scalar(255, 0, 0), 2);
        Imgproc.rectangle(mat, RIGHTBOX, new Scalar(255, 0, 0), 2);
        if (leftValue > centerValue && leftValue > rightValue) {
            shippingHubLevel = 1;
            Imgproc.rectangle(mat, LEFTBOX, new Scalar(0, 0, 255), 2);
        } else if (centerValue > leftValue && centerValue > rightValue) {
            shippingHubLevel = 2;
            Imgproc.rectangle(mat, CENTERBOX, new Scalar(0, 0, 255), 2);
        }
        else if (rightValue > leftValue && rightValue > centerValue) {
            shippingHubLevel = 3;
            Imgproc.rectangle(mat, RIGHTBOX, new Scalar(0, 0, 255), 2);
        }
        return mat;
    }
}
