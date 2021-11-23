package org.firstinspires.ftc.Team19567;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class tsePipeline extends OpenCvPipeline
{

    private Mat greyFrame = null;
    @Override
    public Mat processFrame(Mat input)
    {

        Imgproc.cvtColor(input,greyFrame,Imgproc.COLOR_RGBA2GRAY);
        return greyFrame;
    }
}
