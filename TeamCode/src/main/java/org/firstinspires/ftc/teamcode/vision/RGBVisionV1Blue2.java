package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RGBVisionV1Blue2 extends OpenCvPipeline {
    boolean viewportPaused = false;

    private volatile boolean[] positions;

    public Scalar x = new Scalar(87,179,232);
    public Scalar low = new Scalar(53.8, 72.3, 126.1), high =new Scalar(121.8, 168.6, 201.2);

    Mat mask, hierarchy = new Mat();

    Telemetry telemetry;
    public RGBVisionV1Blue2(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Process frame
        mask = new Mat(input.rows(), input.cols(), CvType.CV_8UC1);

        Core.inRange(input, low, high, mask);

        Imgproc.GaussianBlur(mask, mask, new Size(5.0, 15.0), 0.00);

        ArrayList<MatOfPoint> cont = new ArrayList<MatOfPoint>();

        Imgproc.findContours(mask, cont, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        Imgproc.drawContours(input, cont,-1, new Scalar(255,0,0), 5);
        return input;

    }

}
