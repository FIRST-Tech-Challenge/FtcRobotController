package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class MarkerDetector extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();

    public MARKER_POSITION position = MARKER_POSITION.UNKNOWN;
    double leftRedAvg;
    private static final int SUBMAT_WIDTH = 80;
    private static final int SUBMAT_HEIGHT = 80;
    private Telemetry telemetry;
    public enum MARKER_POSITION {
        LEFT, RIGHT, CENTER, UNDETECTED, UNKNOWN
    }

    public MarkerDetector(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            telemetry.addLine("working matrix is empty");
            telemetry.update();
            return input;
        }
        telemetry.addLine("working matrix is not empty -- drawing");
        telemetry.update();


        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

        Mat matLeft = workingMatrix.submat(140, 220, 0, 80); //frame is 240x320
        Mat matCenter = workingMatrix.submat(140, 220, 120, 200);
        Mat matRight = workingMatrix.submat(140, 220, 240, 320);

        Imgproc.rectangle(workingMatrix, new Rect(00, 140, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(120, 140, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(240, 140, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));

        double leftRedTotal = Core.sumElems(matLeft).val[1];
        double rightRedTotal = Core.sumElems(matRight).val[1];
        double centerRedTotal = Core.sumElems(matCenter).val[1];

        Log.d("vision", "processFrame: left sum - " + leftRedTotal);
        Log.d("vision", "processFrame: right sum - " + rightRedTotal);
        Log.d("vision", "processFrame: center sum - " + centerRedTotal);

        double leftBlueTotal = Core.sumElems(matLeft).val[2];
        double rightBlueTotal = Core.sumElems(matRight).val[2];
        double centerBlueTotal = Core.sumElems(matCenter).val[2];

        leftRedAvg = leftRedTotal / (SUBMAT_WIDTH * SUBMAT_HEIGHT);
        double rightRedAvg = rightRedTotal / (SUBMAT_WIDTH * SUBMAT_HEIGHT);
        double centerRedAvg = centerRedTotal / (SUBMAT_WIDTH * SUBMAT_HEIGHT);

        Log.d("vision", "processFrame: left avg - " + leftRedAvg);
        Log.d("vision", "processFrame: right avg - " + rightRedAvg);
        Log.d("vision", "processFrame: center avg - " + centerRedAvg);

        double avgLeftCb = leftBlueTotal / (SUBMAT_WIDTH * SUBMAT_HEIGHT);
        double avgRightCb = rightBlueTotal / (SUBMAT_WIDTH * SUBMAT_HEIGHT);
        double avgCenterCb = centerBlueTotal / (SUBMAT_WIDTH * SUBMAT_HEIGHT);

        double biggestRedAvg = bigAbsVal(leftRedAvg, rightRedAvg, centerRedAvg);

        if (biggestRedAvg == leftRedAvg) {
            position = MARKER_POSITION.LEFT;
        } else if (biggestRedAvg == rightRedAvg) {
            position = MARKER_POSITION.RIGHT;
        } else if (biggestRedAvg == centerRedAvg) {
            position = MARKER_POSITION.CENTER;
        } else {
            telemetry.addLine("biggest avg is not left right or center");
        }

        Log.d("vision", "processFrame: position - " + position);

        //return
        return workingMatrix;
        }

    public double bigAbsVal (double a, double... others) {

        double max = a;

        for (double next : others) {
            if (Math.abs(next) > Math.abs(max)) {
                max = next;
            }
        }
        return max;
    }
}



