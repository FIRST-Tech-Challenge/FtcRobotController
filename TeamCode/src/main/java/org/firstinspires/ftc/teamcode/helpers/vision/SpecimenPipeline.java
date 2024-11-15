package org.firstinspires.ftc.teamcode.helpers.vision;

import android.graphics.Canvas;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SpecimenPipeline extends OpenCvPipeline {
    private final Telemetry telemetry;

    public SpecimenPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.rectangle(input, new Point(20.0, 20.0), new Point(40.0, 40.0), new Scalar(250, 10, 10));
        return input;
    }
}
