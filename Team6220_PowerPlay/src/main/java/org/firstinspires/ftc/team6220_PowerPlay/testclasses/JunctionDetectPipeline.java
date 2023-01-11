package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import org.opencv.core.Mat;

public class JunctionDetectPipeline extends ColorDetectPipeline {

    @Override
    public Mat processFrame(Mat input) throws IllegalArgumentException {
        if (input == null) {
            throw new IllegalArgumentException("Input cannot be null");
        }

        int[] ca = {30, 255, 255};
        int[] co = {30, 200, 200};

        return processFrameWithRange(input, ca, co);
    }
}
