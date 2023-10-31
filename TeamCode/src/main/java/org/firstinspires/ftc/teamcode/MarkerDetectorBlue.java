package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class MarkerDetectorBlue extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();

    public MARKER_POSITION position = MARKER_POSITION.UNKNOWN;
    private static final int SUBMAT_WIDTH = 160;
    private static final int SUBMAT_HEIGHT = 160;
    private Telemetry telemetry;

    public enum MARKER_POSITION {
        LEFT, RIGHT, CENTER, UNDETECTED, UNKNOWN;
    }

    public MarkerDetectorBlue(Telemetry telemetry) {

        this.telemetry = telemetry;
    }

    @Override
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            Log.d("vision", "processFrame: empty working matrix");
            return input;
        }

        telemetry.update();

        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

        Mat matLeft = workingMatrix.submat(160, 320, 0, 160); //frame is 240x320 480x640
        Mat matCenter = workingMatrix.submat(160, 320, 240, 400);
        Mat matRight = workingMatrix.submat(160, 320, 480, 640);

        Imgproc.rectangle(workingMatrix, new Rect(0, 160, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(240, 160, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(480, 160, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));

        double leftCrTotal = Core.sumElems(matLeft).val[1];
        double rightCrTotal = Core.sumElems(matRight).val[1];
        double centerCrTotal = Core.sumElems(matCenter).val[1];

        double leftCbTotal = Core.sumElems(matLeft).val[2];
        double rightCbTotal = Core.sumElems(matRight).val[2];
        double centerCbTotal = Core.sumElems(matCenter).val[2];

        double avgLeftCr = leftCrTotal / (SUBMAT_WIDTH * SUBMAT_HEIGHT);
        double avgRightCr = rightCrTotal / (SUBMAT_WIDTH * SUBMAT_HEIGHT);
        double avgCenterCr = centerCrTotal / (SUBMAT_WIDTH * SUBMAT_HEIGHT);

        double avgLeftCb = leftCbTotal / (SUBMAT_WIDTH * SUBMAT_HEIGHT);
        double avgRightCb = rightCbTotal / (SUBMAT_WIDTH * SUBMAT_HEIGHT);
        double avgCenterCb = centerCbTotal / (SUBMAT_WIDTH * SUBMAT_HEIGHT);

        position = MARKER_POSITION.UNDETECTED;

        // Log.d("vision", "Avg  left = " + avgLeftCr + " center = " + avgCenterCr + " right = " + avgRightCr);
        if (avgLeftCb > avgCenterCb) {
            if (avgLeftCb > avgRightCb) {
                if (((160 <= avgLeftCb) && (avgLeftCb <= 240)) && ((avgLeftCr >= 16) && (avgLeftCr <= 128))) {
                    position = MARKER_POSITION.LEFT;
                }
            } else {
                if (((160 <= avgRightCb) && (avgRightCb <= 240)) && ((avgRightCr >= 16) && (avgRightCr <= 128))) {
                    position = MARKER_POSITION.RIGHT;
                }
            }
        } else {
            if (avgCenterCb > avgRightCb) {
                if (((160 <= avgCenterCb) && (avgCenterCb <= 240)) && ((avgCenterCr >= 16) && (avgCenterCr <= 128))) {
                    position = MARKER_POSITION.CENTER;
                }
            } else {
                if (((160 <= avgRightCb) && (avgRightCb <= 240)) && ((avgRightCr >= 16) && (avgRightCr <= 128))) {
                    position = MARKER_POSITION.RIGHT;
                }
            }
        }
        return input;
        //return workingMatrix;
    }
}





