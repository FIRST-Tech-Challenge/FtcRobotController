package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import org.opencv.core.Mat;

public class JunctionDetectPipeline extends ColorDetectPipeline {
    public JunctionDetectPipeline() {
        super(new int[] {35, 127, 127}, new int[] {100, 255, 255});
    }
}
