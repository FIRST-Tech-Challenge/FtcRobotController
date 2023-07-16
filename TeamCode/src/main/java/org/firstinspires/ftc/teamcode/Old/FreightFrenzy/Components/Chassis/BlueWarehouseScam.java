package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Chassis;


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

public class BlueWarehouseScam extends OpenCvPipeline {
    LinearOpMode op;
    Telemetry telemetry;
    int width = 320, height = 240;
    double[] pos = {0, 0};
    ArrayList<double[]> positions = new ArrayList<double[]>();
    Mat mat = new Mat();
    double camheight =9;

/* Nathan's values
    static final Rect LEFT_ROI = new Rect(
            new Point(-50, 50), //
            new Point( 50,150));
    static final Rect MIDDLE_ROI = new Rect(
            new Point(200,350),
            new Point(300 ,400));
    static final Rect RIGHT_ROI = new Rect(
            new Point(500,  575),
            new Point(600,  675));*/

    //New calculations

    static double PERCENT_COLOR_THRESHOLD = 0.4; //percentage of color

    public BlueWarehouseScam() {
        telemetry = op.telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        ArrayList<double[]> positionsBeta = new ArrayList<>();
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        // if something is wrong, we assume there's no skystone
        if (mat.empty()) {

            return input;
        }

        // We create a HSV range for yellow to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        Scalar lowHSV = new Scalar(10, 100, 100); // lower bound HSV for yellow
        Scalar highHSV = new Scalar(30, 255, 255); // higher bound HSV for yellow
        Mat thresh = new Mat();

        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(mat, lowHSV, highHSV, thresh);
        Mat test = new Mat();
        thresh.copyTo(test);
        Core.bitwise_and(input,input, thresh, test);
        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis
        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 50, 200,3, true);

        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        // We then find the bounding rectangles of those contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        RotatedRect[] rectangle = new RotatedRect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            rectangle[i] = Imgproc.minAreaRect(contoursPoly[i]);
        }
        boolean[] good = new boolean[contours.size()];
        for (int i = 0; i < good.length; i++) {
            good[i] = false;
        }
        for (int i = 0; i != boundRect.length; i++) {
            if (!good[i] && hierarchy.get(0, i)[2] != -1) {
                ArrayList<Integer> children = new ArrayList<Integer>();
                int currentChild = (int) hierarchy.get(0, i)[2];
                double childArea = rectangle[currentChild].size.area();
                children.add(currentChild);
                while (hierarchy.get(0, currentChild)[0] != -1) {
                    currentChild = (int) hierarchy.get(0, currentChild)[0];
                    childArea += rectangle[currentChild].size.area();
                    children.add(currentChild);
                }
                if (childArea > 1.0 / 2.0 * rectangle[i].size.area()) {
                    for (int k = 0; k < children.size(); k++) {
                        good[k] = true;
                    }
                }
            } else if (!good[i] && hierarchy.get(0, i)[3] != -1) {
                ArrayList<Integer> bruddahs = new ArrayList<Integer>();
                int currentBruddah = i;
                double bruddahArea = rectangle[currentBruddah].size.area();
                bruddahs.add(currentBruddah);
                while (hierarchy.get(0, currentBruddah)[0] != -1) {
                    currentBruddah = (int) hierarchy.get(0, currentBruddah)[0];
                    bruddahArea += rectangle[currentBruddah].size.area();
                    bruddahs.add(currentBruddah);
                }
                if (bruddahArea > 1.0 / 2.0 * rectangle[(int) hierarchy.get(0, i)[3]].size.area()) {
                    for (int k = 0; k < bruddahs.size(); k++) {
                        good[k] = true;
                    }
                }
            } else {
                good[i] = true;
            }
            // draw red bounding rectangles on mat
            // the mat has been converted to HSV so we need to use HSV as well
            if (good[i]&&rectangle[i].size.width>30&&rectangle[i].size.height>30&&rectangle[i].size.width/rectangle[i].size.height<1.5&&rectangle[i].size.width/rectangle[i].size.height>0.66) {
                Imgproc.rectangle(thresh, boundRect[i], new Scalar(0.1, 100, 100), 10);
                double sizer = sqrt(pow(rectangle[i].size.width, 2) + pow(rectangle[i].size.height, 2));
                MatOfPoint currentContour = contours.get(i);
                //sizer*=Imgproc.contourArea(currentContour)/rectangle[i].size.area();
                positionsBeta.add(positionProcessor(sizer, new double[]{rectangle[i].center.x, rectangle[i].center.y}));
//                if (rectangle[i].size.width > rectangle[i].size.height * 2) {
//                    sizer = 1.3*rectangle[i].size.height;
//                    positionsBeta.add(positionProcessor(sizer, new double[]{rectangle[i].center.x + (rectangle[i].size.width - rectangle[i].size.height) / 2, rectangle[i].center.y}));
//                } else if (rectangle[i].size.width * 2 < rectangle[i].size.height) {
//                    sizer = sqrt(2) * rectangle[i].size.width;
//                    positionsBeta.add(positionProcessor(sizer, new double[]{rectangle[i].center.x, rectangle[i].center.y}));
//                } else {
//                    positionsBeta.add(positionProcessor(sizer, new double[]{rectangle[i].center.x, rectangle[i].center.y}));
//                }
            }
        }

        Mat mat2 = new Mat();
        Imgproc.cvtColor(input, mat2, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(mat2, mat2, Imgproc.COLOR_HSV2RGB);

        // We create a HSV range for yellow to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        Scalar lowRGB = new Scalar(230, 230, 230); // lower bound RGB for white
        Scalar highRGB = new Scalar(255, 255, 255); // higher bound RGB for white
        Mat thresh2 = new Mat();

        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(mat2, lowRGB, highRGB, thresh2);

        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis
        Mat edges2 = new Mat();
        Mat test2 = new Mat();
        thresh2.copyTo(test2);
        Core.bitwise_and(input,input, thresh2, test2);
        Imgproc.Canny(thresh2, edges2, 50, 200,3,true);

        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        // We then find the bounding rectangles of those contours
        List<MatOfPoint> contours2 = new ArrayList<>();
        Mat hierarchy2 = new Mat();
        Imgproc.findContours(edges2, contours2, hierarchy2, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        MatOfPoint2f[] contoursPoly2 = new MatOfPoint2f[contours2.size()];
        Rect[] boundRect2 = new Rect[contours2.size()];
        Point center[] = new Point[contours2.size()];
        float radius[][] = new float[contours2.size()][1];
        RotatedRect[] rectangle2 = new RotatedRect[contours2.size()];
        for (int i = 0; i < contours2.size(); i++) {
            contoursPoly2[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours2.get(i).toArray()), contoursPoly2[i], 3, true);
            boundRect2[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly2[i].toArray()));
            if (center[i] == null || radius[i] == null) {
                center[i] = new Point(0, 0);
                radius[i][0] = 0;
            }
            Imgproc.minEnclosingCircle(contoursPoly2[i], center[i], radius[i]);
            rectangle2[i] = Imgproc.minAreaRect(contoursPoly2[i]);
        }
            // draw red bounding rectangles on mat
            // the mat has been converted to HSV so we need to use HSV as well
        for(int i=0;i<boundRect2.length;i++){
            if (rectangle2[i].size.width>30&&rectangle2[i].size.height>30&&rectangle2[i].size.width/rectangle2[i].size.height<2.5&&rectangle2[i].size.width/rectangle2[i].size.height>0.4) {
                positionsBeta.add(positionProcessor(radius[i][0]*3, new double[]{center[i].x,center[i].y}));
                Imgproc.circle(thresh2, center[i], (int)radius[i][0],new Scalar(0.1, 100, 100), 10);
            }
        }
        //mat.release();
        mat2.release();
        edges.release();
        edges2.release();
        thresh.release();
        thresh2.release();
        hierarchy.release();
        hierarchy2.release();
        test.release();
        test2.release();
        positions = positionsBeta;//evens only
        return mat;
    }

    public ArrayList<double[]> getLocation() {
        return positions;
    }

    public double[] positionProcessor(double size, double[] midpoynt) {
        double[] position = {(double) size, (double) 1000};
        float distconst = (float) (24 / 14.8 * 24 * 24 / .7514 * 22 / 18.74); // converts pixels to inches
        float angleconst = (float) (.1189 * 14.0 / 11.65);// converts pixels to angle
        double dist = ((float) distconst / size);
        if (dist > 9) {
            dist = sqrt(dist * dist - 9 * 9);
            float angule = (float) (midpoynt[0] - 160) * angleconst;
            double opposite = (float) sin(angule * PI / 180) * dist; // the x
            double adjacent = (float) cos(angule * PI / 180) * dist; // the y
            position[1] = adjacent;
            position[0] = opposite;
        }
        return position;
    }
}
//24.298, 28.87   sin(p*a)=8/24.3    7/24.3   sin(-147*a)*24.3=8 a=arcsin(8/24.3)/-147

//18.4349,-120.298
//follow this tutorial "FTC EasyOpenCV Tutorial + SkyStone Example"

//https://www.youtube.com/watch?v=JO7dqzJi8lw&t=400s

//https://gist.github.com/oakrc/12a7b5223df0cb55d7c1288ce96a6ab7

