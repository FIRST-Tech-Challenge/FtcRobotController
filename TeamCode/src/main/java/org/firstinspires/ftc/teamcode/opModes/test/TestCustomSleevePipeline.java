package org.firstinspires.ftc.teamcode.opModes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

// never ended up being used

@Disabled
class TestCustomSleevePipeline extends OpenCvPipeline
{
    @Override
    public Mat processFrame(Mat input)
    {
        return input;
    }
}