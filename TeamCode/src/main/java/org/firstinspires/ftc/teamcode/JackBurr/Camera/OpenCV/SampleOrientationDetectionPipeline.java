package org.firstinspires.ftc.teamcode.JackBurr.Camera.OpenCV;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SampleOrientationDetectionPipeline extends OpenCvPipeline {
    public double angle = 0.0;
    @Override
    public Mat processFrame(Mat input) {
        angle = estimateSampleOrientation(input);
        return input;
    }

    public double estimateSampleOrientation(Mat input) {

        Mat gray = new Mat();
        Mat blur = new Mat();

        Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);

        Imgproc.GaussianBlur(gray, blur, new Size(5, 5), 0);

        Mat edges = new Mat();

        Imgproc.Canny(gray, edges, 50, 150);

        List<MatOfPoint> contours = new ArrayList<>();

        Mat hierarchy = new Mat();

        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        angle = 0.0;

        for (MatOfPoint contour : contours) {

            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));

            angle = rect.angle;

            if (rect.size.width < rect.size.height) {

                angle += 90;

            }

            Point[] boxPoints = new Point[4];

            rect.points(boxPoints);

            for (int i = 0; i < 4; i++) {

                Imgproc.line(input, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(0, 255, 0), 2);

            }

            break;

        }

        return angle;

    }


}
