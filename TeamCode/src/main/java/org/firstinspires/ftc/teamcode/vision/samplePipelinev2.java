package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class samplePipelinev2 extends OpenCvPipeline {
    public Scalar lowerFilter = new Scalar(0,0,0);
    public Scalar upperFilter = new Scalar(255,255,255);

    private Mat YCrCb = new Mat();
    private Mat binary = new Mat();
    private Mat output = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(YCrCb, lowerFilter, upperFilter, binary);
        output.release();
        Core.bitwise_and(input, input, output, binary);
        return output;
    }
}
