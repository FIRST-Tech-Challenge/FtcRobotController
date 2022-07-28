package org.firstinspires.ftc.masters;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.Video;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class DenseOpticalFlowPipeline extends OpenCvPipeline {

    public RotatedRect rot_rect;
    private int CAMERA_WIDTH;
    private int CAMERA_HEIGHT;

    // hardcode the initial location of window

    Mat frame1 = new Mat() , prvs = new Mat();

    Telemetry telemetry;

    public DenseOpticalFlowPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        CAMERA_WIDTH = input.width();
        CAMERA_HEIGHT = input.height();

        if (frame1 == null) {
            frame1 = input;
            Imgproc.cvtColor(frame1, prvs, Imgproc.COLOR_BGR2GRAY);

        } else {

            Mat frame2 = new Mat(), next = new Mat();
            frame2 = input;
            Imgproc.cvtColor(frame2, next, Imgproc.COLOR_BGR2GRAY);

            Mat flow = new Mat(prvs.size(), CvType.CV_32FC2);
            Video.calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
            // visualization
            ArrayList<Mat> flow_parts = new ArrayList<>(2);
            Core.split(flow, flow_parts);
            Mat magnitude = new Mat(), angle = new Mat(), magn_norm = new Mat();
            Core.cartToPolar(flow_parts.get(0), flow_parts.get(1), magnitude, angle, true);
            Core.normalize(magnitude, magn_norm, 0.0, 1.0, Core.NORM_MINMAX);
            float factor = (float) ((1.0 / 360.0) * (180.0 / 255.0));
            Mat new_angle = new Mat();
            Core.multiply(angle, new Scalar(factor), new_angle);
            //build hsv image
            ArrayList<Mat> _hsv = new ArrayList<>();
            Mat hsv = new Mat(), hsv8 = new Mat(), bgr = new Mat();
            _hsv.add(new_angle);
            _hsv.add(Mat.ones(angle.size(), CvType.CV_32F));
            _hsv.add(magn_norm);
            Core.merge(_hsv, hsv);
            hsv.convertTo(hsv8, CvType.CV_8U, 255.0);
            Imgproc.cvtColor(hsv8, bgr, Imgproc.COLOR_HSV2BGR);
            prvs = next;
        }

        return input;
    }
}