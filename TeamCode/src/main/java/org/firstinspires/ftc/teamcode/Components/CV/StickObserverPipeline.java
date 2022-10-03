package org.firstinspires.ftc.teamcode.Components.CV;

import static java.lang.Math.PI;
import static java.lang.Math.sin;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class StickObserverPipeline extends OpenCvPipeline {
    int width = 320, height = 240;
    //-1.3182 , 16.007
    //155, 397
    double centerOfPole = 0, poleSize = 0, degPerPix = -1.3182/197, widTimesDist = 16.007*397;
    ArrayList<double[]> frameList;


    public StickObserverPipeline() {
        frameList=new ArrayList<>();
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }
        Scalar lowHSV = new Scalar(22, 75, 60); // lower bound HSV for yellow
        Scalar highHSV = new Scalar(40, 255, 255); // higher bound HSV for yellow

        Mat thresh = new Mat();


        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(mat, lowHSV, highHSV, thresh);

        Mat test = new Mat();
        thresh.copyTo(test);
        Core.bitwise_and(input, input, thresh, test);
        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis
        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 200, 3, false);

        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        // We then find the bounding rectangles of those contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(test, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        RotatedRect[] rectangle = new RotatedRect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 5, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            rectangle[i] = Imgproc.minAreaRect(contoursPoly[i]);
        }
        int maxAreaIndex = 0;
        for (int i = 0; i < rectangle.length; i++) {
            if (rectangle[i].size.width > rectangle[maxAreaIndex].size.width) {
                maxAreaIndex = i;
            }
        }
        centerOfPole = rectangle[maxAreaIndex].center.x+sin(rectangle[maxAreaIndex].angle)*rectangle[maxAreaIndex].size.height/2 - 320;
        poleSize = rectangle[maxAreaIndex].size.width;
        frameList.add(new double[]{centerOfPole,poleSize});
//        if(frameList.size()>5) {
//            frameList.remove(0);
//        }
//        input.release();
        mat.release();
        edges.release();
        thresh.copyTo(input);
        thresh.release();
        hierarchy.release();
        test.release();
        return input;
    }

    public double centerOfPole() {
        //256.227,257.307,252.9,253.414: 4,11.75|| 18.8 .073
        //2.5,12.75: 162.7,161.6, 161.7,162.5||  11.09 .068
        //2,9.5 : 187.45, 187.26|| 11.88  .0648
         //1,8,7.8 : 273 || 12.99 .047
        //10.6,22.2 :
        //4.1,20.6 :
        double average=0;
        for(int i=0;i<frameList.size();i++){
            average+=frameList.get(i)[0];
        }
        return average/frameList.size();
    }

    public double poleSize() {
        double average=0;
        for(int i=0;i<frameList.size();i++){
            average+=frameList.get(i)[1];
        }
        return average/frameList.size();
    }

    public double[] poleRotatedPolarCoordDelta() {
        return new double[]{degPerPix * centerOfPole() * PI / 180, widTimesDist / poleSize()};
    }
}