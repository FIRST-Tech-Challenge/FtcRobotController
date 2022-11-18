package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorDetection extends OpenCvPipeline {
    public double[] centerColor;

    @Override
    public Mat processFrame(Mat input) {
        int rows = input.rows();
        int cols = input.cols();

        Mat output = new Mat();
        Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2YCrCb);
        centerColor = output.get(rows/2, cols/2);
        output.release();

        Imgproc.line(input, new Point(0, cols/2f), new Point(rows, cols/2f), new Scalar(0, 255, 0));
        Imgproc.line(input, new Point(rows/2f, 0), new Point(rows/2f, cols), new Scalar(0, 255, 0));

        return input;
    }
}
