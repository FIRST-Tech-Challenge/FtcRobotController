package org.firstinspires.ftc.teamcode.Pipelines;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class DuckDetectionPipeline extends OpenCvPipeline{
    double ducc_x = -1;
    double ducc_y = -1;
    double distance = -1;
    double angle = -1;

    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));


    static final int CB_CHAN_IDX = 2;

    @Override
    public Mat processFrame(Mat input) {
        return analyze(input);
    }
    public Mat analyze(Mat input){

        Core.extractChannel(input, input, CB_CHAN_IDX);
        Imgproc.threshold(input, input, 10, 255, Imgproc.THRESH_BINARY_INV);

        Imgproc.dilate(input, input, dilateElement);
        Imgproc.medianBlur(input, input, 11);

        Imgproc.Canny(input, input, 50, 250, 3, true);
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(input, contours, new Mat(), Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        Point[] centers = new Point[contours.size()];
        float[][] radius = new float[contours.size()][1];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            centers[i] = new Point();
            Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);
        }

        Mat drawing = Mat.zeros(input.size(), CvType.CV_8UC3);
        List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
        for (MatOfPoint2f poly : contoursPoly) {
            contoursPolyList.add(new MatOfPoint(poly.toArray()));
        }
        for (int i = 0; i < contours.size(); i++) {
            Scalar color = new Scalar(255,555,0);
            Imgproc.drawContours(drawing, contoursPolyList, i, color);
            Point p1 = boundRect[i].tl();
            Point p2 = boundRect[i].br();
            if(p1.y > input.rows()/2 && p2.y > input.rows()/2)// check for location
                if((p2.x-p1.x > 20 && p2.y-p1.y > 20) && Math.abs(p2.x-p1.x)/Math.abs(p2.y-p1.y) > 0.86){
                    Imgproc.rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);
                    ducc_x = (p2.x-p1.x)/2;
                    ducc_y = (p2.y-p1.y)/2;
                    angle = ((ducc_x-drawing.cols()/2)/(drawing.cols()/2))*30;
                    distance = (p2.x-p1.x);
                }
        }

        Imgproc.line(drawing, new Point(0,input.rows()/2),new Point(input.cols(),input.rows()/2), new Scalar(50,200,200), 1, Imgproc.LINE_AA, 0);

        return drawing;
    }



    public double getDucc_x(){
        return ducc_x;
    }
    public double getDucc_y(){
        return ducc_y;
    }
    public double getAngle(){ return angle; }
    public double getDistance(){ return distance;}
}