package org.firstinspires.ftc.teamcode.Camera.Childrens.PipeLines;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class ConvertToGreyPipeline extends OpenCvPipeline
{
    // Notice this is declared as an instance variable (and re-used), not a local variable
    Mat grey = new Mat();

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);
        return grey;
    }
}