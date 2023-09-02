package org.firstinspires.ftc.masters;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class CVPipelineWMICodeAlongEmpty extends OpenCvPipeline {

    public CVPipelineWMICodeAlongEmpty() {

    }

    @Override
    public Mat processFrame(Mat input) {

        return input;
    }
}