package org.firstinspires.ftc.teamcode.Components;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class StickObserver extends OpenCvPipeline {
    int width = 320, height = 240;
    double centerOfPole = -1000, poleSize = -1000, centerAverage = 0, sizeAverage=0;
    int numberOfFrames = 0;
    double[] acceptedRangeSize = {0,200}, acceptedRangeCenter = {-100,100};


    public StickObserver() {

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
        Mat thresh2 = new Mat();


        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(mat, lowHSV, highHSV, thresh);

        Mat test = new Mat();
        thresh.copyTo(test);
        Core.bitwise_and(input,input, thresh, test);
        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis
        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges,100,200,3,false);

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
        for(int i=0;i<rectangle.length;i++){
            if(rectangle[i].size.height>50&&hierarchy.get(0,i)[2]==-1){
                if(rectangle[i].size.width>rectangle[maxAreaIndex].size.width){
                    maxAreaIndex=i;
                }
            }
            else if(i==maxAreaIndex){
                maxAreaIndex++;
            }
        }
        if(maxAreaIndex!=rectangle.length) {
            centerOfPole = rectangle[maxAreaIndex].center.x-320;
            poleSize = rectangle[maxAreaIndex].size.width;
            if(centerOfPole<acceptedRangeCenter[1]&&centerOfPole>acceptedRangeCenter[0]&&poleSize>acceptedRangeSize[0]&&poleSize<acceptedRangeSize[1]){
                if(numberOfFrames==0){
                    centerAverage = centerOfPole;
                    sizeAverage = poleSize;
                    numberOfFrames++;
                }
                else{
                    numberOfFrames++;
                    centerAverage = (centerAverage*(numberOfFrames-1) + centerOfPole)/numberOfFrames;
                    sizeAverage = (sizeAverage*(numberOfFrames-1) + poleSize)/numberOfFrames;
                }

            }
        }

        mat.release();
//        edges.release();
        thresh.release();
        hierarchy.release();
        test.release();
        return edges;
    }

    public double centerOfPole() {
        return centerOfPole;
    }
    public double poleSize(){
        return poleSize;
    }
}