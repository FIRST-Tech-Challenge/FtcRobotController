package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class MarkerProcessor implements VisionProcessor {
    private final Mat workingMatrix = new Mat();
    private static final int SUBMAT_WIDTH = 80;
    private static final int SUBMAT_HEIGHT = 80;
    public MARKER_POSITION position = MARKER_POSITION.UNKNOWN;
    double avgLeftCr;
    double leftCrTotal;

    public enum MARKER_POSITION {
        LEFT, RIGHT, CENTER, UNDETECTED, UNKNOWN
    }

    private final Telemetry telemetry;
    private final MarkerDetector detector;

    public MarkerProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
        detector = new MarkerDetector(telemetry);
    }

    public MarkerDetector.MARKER_POSITION getPosition() {
        return detector.position;
    }

    @Override
    public void init(int width, int height, final CameraCalibration calibration) {
        // anything to init?
    }

    @Override
    public Object processFrame(final Mat input, long captureTimeNanos) {
        input.copyTo(workingMatrix);
        return detector.processFrame(input);
    }

    @Override
    public void onDrawFrame(final android.graphics.Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, final Object userContext) {
        // draw nothing
        Imgproc.rectangle(workingMatrix, new Rect(0, 80, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(120, 80, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(240, 80, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));
    }
}
