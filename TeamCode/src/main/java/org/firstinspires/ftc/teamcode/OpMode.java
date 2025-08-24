package org.firstinspires.ftc.teamcode;

import org.openftc.easyopencv.OpenCvPipeline;

public class OpMode {
    OpenCvPipeline pipeline;

    public OpMode(OpenCvPipeline pipeline) {
        this.pipeline = pipeline;
    }

    public void init() {
        System.out.println("OpMode Initialized");
    }

    public void loop() {
        if (pipeline instanceof org.firstinspires.ftc.teamcode.vision.yellow) {
            org.firstinspires.ftc.teamcode.vision.yellow yellowPipeline = (org.firstinspires.ftc.teamcode.vision.yellow) pipeline;
            if (yellowPipeline.width > 0) {
                System.out.println("Yellow detected at (" + yellowPipeline.cX + ", " + yellowPipeline.cY + ")");
            } else {
                System.out.println("No yellow detected.");
            }
        }
    }

    public void stop() {
        System.out.println("OpMode Stopped");
    }
}
