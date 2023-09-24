package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TestDetector extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();

    public String position = "LEFT";

    public TestDetector() {

    }

    @Override
    public final Mat processFrame (Mat input) {
        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }

        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

        Mat matLeft = workingMatrix.submat(120, 150, 10, 50); //frame is 240x360
        Mat matCenter = workingMatrix.submat(120, 150, 80, 120);
        Mat matRight = workingMatrix.submat(120, 150, 150, 190);

        Imgproc.rectangle(workingMatrix, new Rect(10, 120, 40, 30), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(80, 120, 40, 30), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(150, 120, 40, 30), new Scalar(0, 255, 0));

        double leftTotal = Core.sumElems(matLeft).val[2];
        double rightTotal = Core.sumElems(matRight).val[2];
        double centerTotal = Core.sumElems(matCenter).val[2];

        if (leftTotal > centerTotal) {
            if (leftTotal > rightTotal) {
                //left is darkest
                position = "LEFT";
            } else {
                //right is darkest
                position = "RIGHT";
            }
        } else {
            if (centerTotal > rightTotal) {
                //center is darkest
                position = "CENTER";
            } else {
                //right is darkest
                position = "RIGHT";
            }
        }

        return workingMatrix;
    }
}
