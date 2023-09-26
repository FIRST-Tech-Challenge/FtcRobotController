package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.IMU;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TestDetector extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();

    public String position = "- - - - -";

    double avgLeftCr;

    double leftCrTotal;
    private static final int SUBMAT_WIDTH = 80;
    private static final int SUBMAT_HEIGHT = 80;

    public TestDetector() {

    }

    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }

        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

        Mat matLeft = workingMatrix.submat(80, 160, 10, 90); //frame is 240x320
        Mat matCenter = workingMatrix.submat(80, 160, 120, 200);
        Mat matRight = workingMatrix.submat(80, 160, 230, 310);

        Imgproc.rectangle(workingMatrix, new Rect(10, 80, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(120, 80, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(230, 80, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));

        leftCrTotal = Core.sumElems(matLeft).val[1];
        double rightCrTotal = Core.sumElems(matRight).val[1];
        double centerCrTotal = Core.sumElems(matCenter).val[1];

        double leftCbTotal = Core.sumElems(matLeft).val[2];
        double rightCbTotal = Core.sumElems(matRight).val[2];
        double centerCbTotal = Core.sumElems(matCenter).val[2];

        avgLeftCr = leftCrTotal / (SUBMAT_WIDTH * SUBMAT_HEIGHT);
        double avgRightCr = rightCrTotal / (SUBMAT_WIDTH * SUBMAT_HEIGHT);
        double avgCenterCr = centerCrTotal / (SUBMAT_WIDTH * SUBMAT_HEIGHT);

        double avgLeftCb = leftCbTotal / (SUBMAT_WIDTH * SUBMAT_HEIGHT);
        double avgRightCb = rightCbTotal / (SUBMAT_WIDTH * SUBMAT_HEIGHT);
        double avgCenterCb = centerCbTotal / (SUBMAT_WIDTH * SUBMAT_HEIGHT);

        // y 16-255
        // cb 16-240
        // cr 16-240
        // cr, cb middle value: 128
        //checking for red values

        // colorMiddleValue = 128;

//        if (((184 <= avgLeftCr) && (avgLeftCr <= 240)) && ((avgLeftCb >= 16) && (avgLeftCb <= 128))) {
//            position = "LEFT";
//        } else if (((184 <= avgRightCr) && (avgRightCr <= 240)) && ((avgRightCb >= 16) && (avgRightCb <= 128))) {
//            position = "RIGHT";
//        } else if (((184 <= avgCenterCr) && (avgCenterCr <= 240)) && ((avgCenterCb >= 16) && (avgCenterCb <= 128))) {
//            position = "CENTER";
//        } else {
//            position = "cannot find marker";
//        }

        position = "cannot find marker";

        if (avgLeftCr > avgCenterCr) {
            if (avgLeftCr > avgRightCr) {
                position = "LEFT";
            } else {
                position = "RIGHT";
            }
        } else {
            if (avgCenterCr > avgRightCr) {
                position = "CENTER";
            } else {
                position = "RIGHT";
            }
        }


//        if (leftCbTotal > centerCbTotal) {
//            if (leftCbTotal > rightCbTotal) {
//                //left is darkest
//                position = "LEFT";
//            } else {
//                //right is darkest
//                position = "RIGHT";
//            }
//        } else {
//            if (centerCbTotal > rightCbTotal) {
//                //center is darkest
//                position = "CENTER";
//            } else {
//                //right is darkest
//                position = "RIGHT";
//            }
//        }

            return workingMatrix;
        }
    }



