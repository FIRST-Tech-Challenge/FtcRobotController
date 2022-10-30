package org.firstinspires.ftc.teamcode.Components.CV;

import static java.lang.Math.PI;
import static java.lang.Math.sin;

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

public class SleeveObserverPipeline extends OpenCvPipeline {
    double centerOfPole = 0, poleSize = 0, degPerPix = -22.5/320, widTimesDist = 16.007*58;
    ArrayList<double[]> frameList;


    static final Rect ROI = new Rect( //130 x 210, 60 x 120
            new Point(130,240),
            new Point(60,150));


    public SleeveObserverPipeline() {
        frameList=new ArrayList<>();
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        //mat turns into HSV value
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        Scalar lowpurpleHSV = new Scalar(20, 70, 80); //lower bound HSV for purple
        Scalar highpurpleHSV = new Scalar(32, 255, 255); //higher bound HSV for purple

        Scalar loworangeHSV = new Scalar(20, 70, 80); //lower bound HSV for orange
        Scalar highorangeHSV = new Scalar(32, 255, 255); //higher bound HSV for orange

        Scalar lowgreenHSV = new Scalar(20, 70, 80); //lower bound HSV for green
        Scalar highgreenHSV = new Scalar(32, 255, 255); //higher bound HSV for green

        Mat thresh = new Mat();

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Mat cone = mat.submat(ROI);

        Core.inRange(mat,lowpurpleHSV,highpurpleHSV,mat);

        double purpleValue = Core.sumElems(cone).val[0]/ROI.area()/255;

        Core.inRange(mat,loworangeHSV,highorangeHSV,mat);

        double orangeValue = Core.sumElems(cone).val[0]/ROI.area()/255;

        Core.inRange(mat,lowgreenHSV,highgreenHSV,mat);

        double greenValue = Core.sumElems(cone).val[0]/ROI.area()/255;

        cone.release();




        //release all the data

        input.release();
        mat.release();
        thresh.release();
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