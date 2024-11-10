package com.kalipsorobotics.code2023;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class MarkerDetector extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();

    public PROP_POSITION position = PROP_POSITION.UNKNOWN;

    double avgLeftCr;
    double avgCenterCr;
    double avgRightCr;
    ALLIANCE_COLOR markerColor;

    double leftCrTotal;
    double avgLeftCb;
    double avgRightCb;
    double avgCenterCb;
    double avgLeftY;
    double avgCenterY;
    double avgRightY;
    private static final int SUBMAT_WIDTH = 120;
    private static final int SUBMAT_HEIGHT = 120;
    private Telemetry telemetry;
    public enum ALLIANCE_COLOR {
        RED, BLUE;
    }

    public enum PROP_POSITION {
        LEFT, RIGHT, CENTER, UNDETECTED, UNKNOWN;
    }

    public MarkerDetector(Telemetry telemetry, ALLIANCE_COLOR color) {
        markerColor = color;
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

        Mat matLeft = workingMatrix.submat(180, 300, 0, 120);
        Mat matCenter = workingMatrix.submat(180, 300, 260, 380);
        Mat matRight = workingMatrix.submat(180, 300, 520, 640);

        Imgproc.rectangle(workingMatrix, new Rect(0, 180, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(260, 180, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(520, 180, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));

        avgLeftY = (Core.sumElems(matLeft).val[0]) / (SUBMAT_WIDTH * SUBMAT_HEIGHT);
        avgCenterY = Core.sumElems(matCenter).val[0] / (SUBMAT_WIDTH * SUBMAT_HEIGHT);
        avgRightY = Core.sumElems(matRight).val[0] / (SUBMAT_WIDTH * SUBMAT_HEIGHT);

        avgLeftCr = (Core.sumElems(matLeft).val[1]) / (SUBMAT_WIDTH * SUBMAT_HEIGHT);
        avgCenterCr = Core.sumElems(matCenter).val[1] / (SUBMAT_WIDTH * SUBMAT_HEIGHT);
        avgRightCr = Core.sumElems(matRight).val[1] / (SUBMAT_WIDTH * SUBMAT_HEIGHT);

        avgLeftCb = (Core.sumElems(matLeft).val[2]) / (SUBMAT_WIDTH * SUBMAT_HEIGHT);
        avgCenterCb = Core.sumElems(matCenter).val[2] / (SUBMAT_WIDTH * SUBMAT_HEIGHT);
        avgRightCb = Core.sumElems(matRight).val[2] / (SUBMAT_WIDTH * SUBMAT_HEIGHT);

        position = PROP_POSITION.UNDETECTED;

        if (markerColor == ALLIANCE_COLOR.RED) {
            if (avgLeftCr > avgCenterCr) {
                if (avgLeftCr > avgRightCr) {
                    if (((160 <= avgLeftCr) && (avgLeftCr <= 240)) && ((avgLeftCb >= 16) && (avgLeftCb <= 128))) {
                        position = PROP_POSITION.LEFT;
                    }
                } else {
                    if (((160 <= avgRightCr) && (avgRightCr <= 240)) && ((avgRightCb >= 16) && (avgRightCb <= 128))) {
                        position = PROP_POSITION.RIGHT;
                    }
                }
            } else {
                if (avgCenterCr > avgRightCr) {
                    if (((160 <= avgCenterCr) && (avgCenterCr <= 240)) && ((avgCenterCb >= 16) && (avgCenterCb <= 128))) {
                        position = PROP_POSITION.CENTER;
                    }
                } else {
                    if (((160 <= avgRightCr) && (avgRightCr <= 240)) && ((avgRightCb >= 16) && (avgRightCb <= 128))) {
                        position = PROP_POSITION.RIGHT;
                    }
                }
            }
        } else {
            if (avgLeftCb > avgCenterCb) {
                if (avgLeftCb > avgRightCb) {
                    if (((120 <= avgLeftCb) && (avgLeftCb <= 240)) && ((avgLeftCr >= 16) && (avgLeftCr <= 138))) {
                        position = PROP_POSITION.LEFT;
                    }
                } else {
                    if (((120 <= avgRightCb) && (avgRightCb <= 240)) && ((avgRightCr >= 16) && (avgRightCr <= 138))) {
                        position = PROP_POSITION.RIGHT;
                    }
                }
            } else {
                if (avgCenterCb > avgRightCb) {
                    if (((120 <= avgCenterCb) && (avgCenterCb <= 240)) && ((avgCenterCr >= 16) && (avgCenterCr <= 138))) {
                        position = PROP_POSITION.CENTER;
                    }
                } else {
                    if (((120 <= avgRightCb) && (avgRightCb <= 240)) && ((avgRightCr >= 16) && (avgRightCr <= 138))) {
                        position = PROP_POSITION.RIGHT;
                    }
                }
            }
        }
        return input;
    }
}




