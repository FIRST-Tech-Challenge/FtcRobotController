package org.firstinspires.ftc.team8923_2020;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Array;
import java.util.ArrayList;


public abstract class OpenCVRingDetector extends OpenCvPipeline {

    private Mat blurred = new Mat();
    private Mat yuv = new Mat();
    private Mat u = new Mat();
    private Mat displayMat = new Mat();
    private ArrayList<Mat> yuvSplit = new ArrayList<>();
    private double mean1, mean2, mean3;


    /*
    @Override
    public Mat processFrame(Mat rgba, Mat gray){

       //Blur image
        Imgproc.GaussianBlur(rgba, blurred, new Size(10, 10), 0);

        //Convert rgb into yuv
        Imgproc.cvtColor(rgba, yuv, Imgproc.COLOR_RGB2YUV);

        //Split
        Core.split(yuv, yuvSplit);

        //Retrieve u from araylist
        u = yuvSplit.get(1);

        Imgproc.threshold(u, u, 90, 255, Imgproc.THRESH_BINARY_INV);

       //create rectangle
       Imgproc.rectangle(displayMat);


        Imgproc.threshold(u, u, 90, 255, Imgproc.THRESH_BINARY);

        return displayMat;

    }

     */




}


