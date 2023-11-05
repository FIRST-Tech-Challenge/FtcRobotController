package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

//TODO: get rid of this class and refactor markerprocessorred
public class MarkerProcessorBlue implements VisionProcessor {

    private Telemetry telemetry;
    private Mat workingMatrix;

    public MarkerProcessorBlue(Telemetry telemetry) {

        this.telemetry = telemetry;
        detector = new MarkerDetectorBlue(telemetry);
    }

    private final MarkerDetectorBlue detector;


    public MarkerDetectorBlue.MARKER_POSITION getPosition() {
        return detector.position;
    }

    @Override
    public void init(int width, int height, final CameraCalibration calibration) {
        // anything to init?
    }

    @Override
    public Object processFrame(final Mat input, long captureTimeNanos) {
        telemetry.addLine("process frame");
        telemetry.update();
        workingMatrix = detector.processFrame(input);
        return workingMatrix;
    }

    @Override
    public void onDrawFrame(final android.graphics.Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, final Object userContext) {
        int SUBMAT_WIDTH = 160;
        int SUBMAT_HEIGHT = 160;

        // draw nothing
        Imgproc.rectangle(workingMatrix, new Rect(0, 160, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(240, 160, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(480, 160, SUBMAT_WIDTH, SUBMAT_HEIGHT), new Scalar(0, 255, 0));
    }
}
