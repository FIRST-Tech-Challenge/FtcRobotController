package org.firstinspires.ftc.teamcode.CVRec;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CVRingSearchPipeline extends CVPipelineBase {

    private Mat region_Cb;
    private Mat YCrCb = new Mat();
    private Mat Cb = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        return null;
    }

    private void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }
}
