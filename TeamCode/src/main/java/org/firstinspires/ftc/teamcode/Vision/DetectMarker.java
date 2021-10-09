package org.firstinspires.ftc.teamcode.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class DetectMarker extends OpenCvPipeline {
    Telemetry telemetry;
    public DetectMarker(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        return null;
    }
}
