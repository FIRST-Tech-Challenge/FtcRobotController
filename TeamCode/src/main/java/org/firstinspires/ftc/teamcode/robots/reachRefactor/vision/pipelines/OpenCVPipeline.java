package org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.pipelines;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.Position;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCVPipeline extends OpenCvPipeline {

    private Position lastPosition;

    @Override
    public Mat processFrame(Mat input) {
        return null;
    }

    public Position getLastPosition() {
        return lastPosition;
    }

    public void reset() {

    }
}
