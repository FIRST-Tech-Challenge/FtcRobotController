package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class SampleDetection  extends OpenCvPipeline implements VisionProcessor {
    public static final int START_X = 160;
    public static final int FINISH_X = 290;
    public static final int START_Y = 380;
    public static final int FINISH_Y = 480;
    public static final int THRESHOLD_FOR_COLOR = 150;
    private final Telemetry telemetry;
    private int redPixels = 0;
    private int yellowPixels = 0;
    private int bluePixels = 0;
    private int nothingPixels = 0;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    public SampleDetection(Telemetry telemetry)
    {
        this.telemetry = telemetry;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return processFrame(frame);
    }

    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(Color.RED);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(scaleCanvasDensity * 4);
        canvas.drawRect(new Rect((int) (START_X * scaleBmpPxToCanvasPx), (int) (START_Y * scaleBmpPxToCanvasPx), (int) (FINISH_X * scaleBmpPxToCanvasPx), (int) (FINISH_Y * scaleBmpPxToCanvasPx)), paint);
    }

    @Override
    public Mat processFrame(Mat input) {
        redPixels = 0;
        yellowPixels = 0;
        bluePixels = 0;
        nothingPixels = 0;
        for (int i = START_X; i < FINISH_X; i++) {
            for (int j = START_Y; j < FINISH_Y; j++) {
                double[] color = input.get(j, i);
                if(color[0] > THRESHOLD_FOR_COLOR && color[1] > THRESHOLD_FOR_COLOR  && color[2] > THRESHOLD_FOR_COLOR )
                {
                    nothingPixels++;
                }else if(color[0] > THRESHOLD_FOR_COLOR)
                {
                    redPixels++;
                }else if(color[1] > THRESHOLD_FOR_COLOR)
                {
                    yellowPixels++;
                }else if(color[2] > THRESHOLD_FOR_COLOR)
                {
                    bluePixels++;
                }else {
                    nothingPixels++;
                }
            }
        }
        return input;
    }

    public Sample getIntakeStatus() {
        if (redPixels > bluePixels && redPixels > yellowPixels && redPixels > nothingPixels) {
            return Sample.RED;
        } else if (bluePixels > redPixels && bluePixels > yellowPixels && bluePixels > nothingPixels) {
            return Sample.BLUE;
        } else if (yellowPixels > redPixels && yellowPixels > bluePixels && yellowPixels > nothingPixels) {
            return Sample.YELLOW;
        } else {
            return Sample.NOTHING;
        }
    }

    public String getIntakeString() {
        Sample intakeStatus = getIntakeStatus();
        if (intakeStatus.equals(IntakeStatus.RED)) {
            return "RED";
        } else if (intakeStatus.equals(IntakeStatus.BLUE)) {
            return "BLUE";
        } else if (intakeStatus.equals(IntakeStatus.YELLOW)) {
            return "YELLOW";
        }
        return "NOTHING";
    }

    public enum IntakeStatus
    {
        RED, BLUE, YELLOW
    }
}
