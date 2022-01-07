package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisionPipeline extends OpenCvPipeline{
    int avg1;
    int avg2;
    int avg3;
    Mat workingMat= new Mat();
    Mat submat1 = new Mat();
    Mat submat2 = new Mat();
    Mat submat3 = new Mat();
    Point point1 = new Point(45,125);
    Point point2 = new Point(65,165);
    Point point3 = new Point(120,135);
    Point point4 = new Point(140,175);
    Point point5 = new Point(200,145);
    Point point6 = new Point(220,185);

    @Override
    public Mat processFrame(Mat input)
    {
        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */

        /*
         * Draw a simple box around the middle 1/2 of the entire frame
         */

        workingMat = input;

        Imgproc.rectangle(workingMat,point1,point2, new Scalar(100,0,0),3);
        Imgproc.rectangle(workingMat,point3,point4, new Scalar(100,0,0),3);
        Imgproc.rectangle(workingMat,point5,point6, new Scalar(100,0,0),3);

        submat1 = workingMat.submat(new Rect(point1,point2));
        submat2 = workingMat.submat(new Rect(point3,point4));
        submat3 = workingMat.submat(new Rect(point5,point6));

        avg1 = (int)Core.mean(submat1).val[0];
        avg2 = (int)Core.mean(submat2).val[0];
        avg3 = (int)Core.mean(submat3).val[0];

        return input;
    }
    public boolean barcodeL(){
        return (avg1>avg2 && avg1>avg3);
    }
    public boolean barcodeC(){
        return (avg2>avg1 && avg2>avg3);
    }
    public boolean barcodeR(){
        return (avg3>avg2 && avg3>avg1);
    }
}