package com.kalipsorobotics.fresh;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class MarkerProcessor implements VisionProcessor {

    private Telemetry telemetry;
    private Mat workingMatrix;

    double avgLeftCb;
    double avgRightCb;
    double avgCenterCb;

    double avgLeftCr;
    double avgCenterCr;
    double avgRightCr;
    double avgLeftY;
    double avgCenterY;
    double avgRightY;

    double counter;

    public MarkerProcessor(Telemetry telemetry, MarkerDetector.ALLIANCE_COLOR allianceColor) {

        this.telemetry = telemetry;
        detector = new MarkerDetector(telemetry, allianceColor);
    }

    private final MarkerDetector detector;


    public MarkerDetector.MARKER_POSITION getPosition() {
        return detector.position;
    }

    @Override
    public void init(int width, int height, final CameraCalibration calibration) {
        // anything to init?
    }

    @Override
    public Object processFrame(final Mat input, long captureTimeNanos) {
        avgLeftCb = detector.avgLeftCb;
        avgCenterCb = detector.avgCenterCb;
        avgRightCb = detector.avgRightCb;
        avgLeftCr = detector.avgLeftCr;
        avgCenterCr = detector.avgCenterCr;
        avgRightCr = detector.avgRightCr;
        avgLeftY = detector.avgLeftY;
        avgCenterY = detector.avgCenterY;
        avgRightY = detector.avgRightY;
        telemetry.addLine("process frame");
//        telemetry.update();
        workingMatrix = detector.processFrame(input);
        counter++;
        Log.d("vision", String.valueOf(counter));
        return workingMatrix;
    }

    @Override
    public void onDrawFrame(final android.graphics.Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, final Object userContext) {
        int SUBMAT_WIDTH = 120;
        int SUBMAT_HEIGHT = 120;

        // draw nothing
        Imgproc.rectangle(workingMatrix, new Rect(0, 180, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(260, 180, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(520, 180, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));
    }
}
