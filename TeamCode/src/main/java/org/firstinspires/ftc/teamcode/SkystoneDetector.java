package org.firstinspires.ftc.teamcode;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.BackgroundSubtractorMOG2;
import static org.opencv.video.Video.createBackgroundSubtractorMOG2;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SkystoneDetector extends OpenCvPipeline {

    // BackgroundSubtractorMOG2 bgsubtractor = createBackgroundSubtractorMOG2();

    //@Override  public void init(Mat firstFrame)  { bgsubtractor.setVarThreshold(10);  }

    @Override  public Mat processFrame(Mat input) {
        Mat bgmask = new Mat();
        Mat temp = new Mat();
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();

        if (input.empty()) { return input; }

        //bgsubtractor.apply(input, bgmask, 0 );
        Imgproc.dilate(input, temp, new Mat());
        Imgproc.erode(temp, temp, new Mat());
        Imgproc.dilate(temp, temp, new Mat());

        Mat edges = new Mat();

        Imgproc.Canny(temp, edges, 100, 300);
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        Mat dst = Mat.zeros(input.size(), CvType.CV_8UC1);
        if (contours.size()==0) {  return input; }

        int idx = 0;     int largestComp = 0;      double maxArea = 0;

        for(idx=0;idx<contours.size();idx++ ) {
            final Mat c =  contours.get(idx);
            double area = Imgproc.contourArea(c);
            if( area > maxArea )   {  maxArea = area; largestComp = idx; }
        }

        Scalar color = new Scalar(0, 0, 255);
        // Imgproc.drawContours( dst, contours, largestComp, color, Imgproc.FILLED, Imgproc.LINE_8, hierarchy );

        return edges;
    }
}
