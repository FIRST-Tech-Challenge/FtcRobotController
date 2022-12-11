package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

abstract public class ConeDetectionPipeline extends OpenCvPipeline
{
    public double distance;
    public double size;
    public int centerX = 240;
    double dist = 0;
    double coneSize = 0;
    public boolean grab = false;

    public Mat maskFrame(Mat input, int[] lower, int[] upper){
        Mat HSV = new Mat();
        Size blurSize = new Size(5,5);
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_BGR2HSV);
        Imgproc.blur(HSV, HSV, blurSize);
        return(HSV);
    }

    @Override
    public Mat processFrame(Mat input, int[] lower, int[] upper)
    {
        Mat mask = maskFrame(input, lower, upper);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        MatOfPoint largestContour = new MatOfPoint();

        double maxVal = 0.0;
        int maxValIdx = 0;
        for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++)
        {
            double contourArea = Imgproc.contourArea(contours.get(contourIdx));
            if (maxVal < contourArea)
            {
                maxVal = contourArea;
                maxValIdx = contourIdx;
            }
        }
        Rect xywh = Imgproc.boundingRect(contours.get(maxValIdx));
        Moments m = Imgproc.moments(contours.get(maxValIdx), false);
        if(m.get_m00() > 0)
        {
            double cX = (m.get_m10() / m.get_m00());
            dist = centerX - cX;
            coneSize = xywh.width * xywh.height;
            if (Math.abs(distance) < 30 && coneSize < 3000)
            {
                grab = true;
            } else
            {
                grab = false;
            }
        }
        return input;
    }
    }
}
