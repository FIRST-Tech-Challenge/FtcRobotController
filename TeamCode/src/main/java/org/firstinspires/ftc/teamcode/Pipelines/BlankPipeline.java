package org.firstinspires.ftc.teamcode.Pipelines;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class BlankPipeline extends OpenCvPipeline {

    /**
     * Simply returns the input frame to display on the console.
     * @param input The input frame as a matrix
     * @return the identical input frame.
     */

    @Override
    public Mat processFrame(Mat input){
    return input;
    }
}
