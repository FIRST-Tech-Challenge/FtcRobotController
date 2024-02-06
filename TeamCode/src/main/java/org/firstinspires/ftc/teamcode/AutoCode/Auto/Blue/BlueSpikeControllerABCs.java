package org.firstinspires.ftc.teamcode.AutoCode.Auto.Blue;

import org.firstinspires.ftc.teamcode.AutoCode.Auto.Spike;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BlueSpikeControllerABCs extends OpenCvPipeline
{
    double PropDensity = 145;
    double DensityThreshold = 1.14;
    //double ThreeRingThreshold = 1.16;
    double Density;
    double Density2;
    double ControlDensity;
    double RatioDensity;
    double RatioDensity2;
    Spike Position;
    int width = 30;
    int hight = 35;
    Mat CutoutMat;
    Mat CutoutMat2;
    Mat CutoutMat3;
    Mat YCrCb = new Mat();
    Mat Cr = new Mat();
    Mat Cb = new Mat();

    Point TopLeftPoint = new Point(140,180); // (0,0) is top left // 47 158
    Point BottomRightPoint = new Point(TopLeftPoint.x + width, TopLeftPoint.y + hight); // reminder to adjust the X val off of ultra sonic distance

    Point TopLeftPoint2 = new Point(289,180); // (0,0) is top left
    Point BottomRightPoint2 = new Point(TopLeftPoint2.x + width, TopLeftPoint2.y + hight);

    Point ControlTopLeft = new Point(215,200); // (0,0) is top left // 47 158
    Point ControlBottomRight = new Point(ControlTopLeft.x + width, ControlTopLeft.y + hight);


    @Override
    public void init (Mat input) {
        Imgproc.cvtColor(input,YCrCb,Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb,Cb,2);
        CutoutMat = Cb.submat(new Rect(TopLeftPoint, BottomRightPoint));
        CutoutMat2 = Cb.submat(new Rect(TopLeftPoint2, BottomRightPoint2));
        CutoutMat3 = Cb.submat(new Rect(ControlTopLeft, ControlBottomRight));

    }
    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input,YCrCb,Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb,Cr,1);
        Core.extractChannel(YCrCb,Cb,2);
        Imgproc.rectangle(
                input,
                TopLeftPoint,
                BottomRightPoint,
                new Scalar(0, 0, 255), 2);
        Density = (int) Core.mean(CutoutMat).val[0];
        Imgproc.rectangle(
                input,
                TopLeftPoint2,
                BottomRightPoint2,
                new Scalar(0, 0, 255), 2);
        Density2 = (int) Core.mean(CutoutMat2).val[0];
        Imgproc.rectangle(
                input,
                ControlTopLeft,
                ControlBottomRight,
                new Scalar(0, 255, 0), 2);
        ControlDensity = (int) Core.mean(CutoutMat3).val[0];
        return input;
    }
    public double GetLastDensity() {
        return Density;
    }
    public double GetLastDensity2() {
        return Density2;
    }
    public double GetLastControlDensity() {
        return ControlDensity;
    }
    public double GetLastRatioDensity() {
        return RatioDensity;
    }
    public double GetLastRatioDensity2() {
        return RatioDensity2;
    }
    public Spike GetPosition() {
        RatioDensity = Density/ControlDensity;
        RatioDensity2 = Density2/ControlDensity;
        if (RatioDensity > DensityThreshold) {
           Position = Spike.A;
        }
        else if (RatioDensity2 > DensityThreshold) {
            Position = Spike.B;
        }
        else if (RatioDensity < DensityThreshold && RatioDensity2 < DensityThreshold){
            Position = Spike.C;
        }
        return Position;
    }
}