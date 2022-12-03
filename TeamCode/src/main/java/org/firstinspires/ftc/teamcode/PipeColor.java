package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;




public class PipeColor extends OpenCvPipeline {

    Telemetry telemetry;
    Mat mat = new Mat();
    Mat matHSV = new Mat();

    static int H_lower = 0; // These H/S/V lower/upper variables represent the part in the
    static int H_upper = 0; // switch case that is added to the end HSV value
    static int S_lower = 80; //
    static int S_upper = 253; //
    static int V_lower = 55; //
    static int V_upper = 250; //


    static int H_Lower; // Whereas these values represent the final
    static int H_Upper; // H/S/V values put through the range function
    static int S_Lower; //
    static int S_Upper; //
    static int V_Lower; //
    static int V_Upper; //
    static double Hue = 0;
    static double Saturation = 0;
    static double Value = 0;
    static int HueSeparation = 0;
    static int SaturationSeparation = 0;
    static int ValueSeparation = 0;








    static final Rect LEFT_ROI = new Rect(
            new Point(30,35),
            new Point(80,75));
    static final Rect MID_ROI = new Rect(
            new Point(100,35),
            new Point(150,75));
    static final Rect RIGHT_ROI = new Rect(
            new Point(170,35),
            new Point(220,75));

    static double PERCENT_COLOR_THRESHOLD = 0.5;
    public static String result;
    public PipeColor(Telemetry t, double H, double S, double V, int HS, int SS, int VS){ // The H/S/V = Hue/Saturation/Value, whereas the added S stands for separation
        telemetry = t;
        Hue = H;
        Saturation = S;
        Value = V;
        HueSeparation = HS;
        SaturationSeparation = SS;
        ValueSeparation = VS;


//        , double H, double S, double V  <-- For the above parameter(s)
//        Hue = H;
//        Saturation = S;
//        Value = V;
//
    }




    @Override
    public Mat processFrame(Mat input) {
//        String filea = "DCIM\\rainbow.jpg";
//        Mat src = Imgcodecs.imread(file);


                H_lower = 117;
                H_upper = 129;


        mat = input;

        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_BGR2HSV);
        ArrayList<Mat> HSVList = new ArrayList<>();
        Core.split(mat,HSVList);
        Imgproc.equalizeHist(HSVList.get(2),HSVList.get(2));
        Core.merge(HSVList,mat);
        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_HSV2BGR);

        Imgproc.cvtColor(mat,matHSV,Imgproc.COLOR_BGR2HSV);

        H_Lower = H_lower + (int)Hue - HueSeparation;
        H_Upper = H_upper + (int)Hue + HueSeparation;
        S_Lower = S_lower + (int)Saturation - SaturationSeparation;
        S_Upper = S_upper + (int)Saturation + SaturationSeparation;
        V_Lower = V_lower + (int)Value - ValueSeparation;
        V_Upper = V_upper + (int)Value + ValueSeparation;

        Scalar low_HSV = new Scalar(H_Lower, S_Lower, V_Lower);
        Scalar high_HSV = new Scalar(H_Upper, S_Upper, V_Upper);




        Core.inRange(matHSV,low_HSV,high_HSV,mat);


        matHSV.release();

        telemetry.update();
        return mat;


    }

    public static String getResult(){
        return result;
    }
    public static double getHueLower(){
        return H_Lower;
    }
    public static double getHueUpper(){
        return H_Upper;
    }
    public static double getSaturationLower(){ return S_Lower; }
    public static double getSaturationUpper(){ return S_Upper; }
    public static double getValueLower(){ return V_Lower; }
    public static double getValueUpper(){ return V_Upper; }




}
