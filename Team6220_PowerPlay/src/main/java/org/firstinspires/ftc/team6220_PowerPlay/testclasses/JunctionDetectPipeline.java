package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import org.opencv.core.Mat;

public class JunctionDetectPipeline extends ColorDetectPipeline {

    @Override
    public Mat processFrame(Mat input) throws IllegalArgumentException {
        if (input == null) {
            throw new IllegalArgumentException("Input cannot be null");
        }

        int[] ca = {43, 255, 255};
        int[] co = {50, 25, 200};

        return processFrameWithRange(input, ca, co);
    }
}
