package org.firstinspires.ftc.teamcode.robotSubSystems.Camera;


import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;

public final class Constants {
    //HSV constants
    public static Scalar redLowHSV = new Scalar(0 , 125 , 35);
    public static Scalar redHighHSV = new Scalar(90 , 255 , 255);

    public static Scalar blueLowHSV = new Scalar(85 , 50 , 115);
    public static Scalar blueHighHSV = new Scalar(130 , 180 , 255);
    //Scalar (color) constants
    public static Scalar Green = new Scalar(0 , 255 ,0);
    public static Scalar White = new Scalar(255 , 255 , 255);
    public static Scalar Red = new Scalar(255 , 0 , 0);
    public static int binary = Imgproc.COLOR_RGB2HSV;

    public static Rect cropRect = new Rect(new Point(0,270),new Point(1280,720));
    //Blur size radius
    public static Size BlurRadius = new Size(37,37);
    //camera
    public static OpenCvCamera camera;
}
