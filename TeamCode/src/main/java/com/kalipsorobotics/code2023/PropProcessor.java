package com.kalipsorobotics.code2023;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

class PropProcessor implements VisionProcessor {

    private Telemetry telemetry;
    private Mat workingMatrix;

    double counter;
    final PropDetector detector;

    public PropProcessor(Telemetry telemetry, PropDetector.ALLIANCE_COLOR allianceColor) {
        Log.d("vision", "processor: constructing started");
        this.telemetry = telemetry;
        detector = new PropDetector(telemetry, allianceColor);
        Log.d("vision", "processor: constructing finished");
    }

    public PropDetector.PROP_POSITION getPosition() {
        Log.d("vision", "processor: getPosition entered");
        return detector.getPropPosition();
    }

    @Override
    public void init(int width, int height, final CameraCalibration calibration) {
        // anything to init?
    }

    @Override
    public Object processFrame(final Mat input, long captureTimeNanos) {
        Log.d("vision", "processor: entered process frame");
        telemetry.addLine("process frame");
        telemetry.update();
        workingMatrix = detector.processFrame(input);
        counter++;
        Log.d("vision", "processor: position is" + detector.getPropPosition());
        Log.d("vision", "processor:" + counter);
        return workingMatrix;
    }

    @Override
    public void onDrawFrame(final android.graphics.Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, final Object userContext) {
        int SUBMAT_WIDTH = 120;
        int SUBMAT_HEIGHT = 120;
        Log.d("vision", "processor: drawing boxes");

        // draw nothing
        Imgproc.rectangle(workingMatrix, new Rect(0, 180, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(260, 180, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(520, 180, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));
    }
}
