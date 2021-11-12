package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class CVClass extends OpenCvPipeline{
    double hue;
    double sensitivity;

    List<MatOfPoint> contours = new ArrayList<>();

    //WHY DO WE NEED SO MANY MATS????!?!?!?!
    Mat hsv = new Mat();
    Mat blur = new Mat();
    Mat output = new Mat();
    Mat singleColor = new Mat();
    Mat hierarchy = new Mat();

    int barcode = 0; //level 1 is bottom, level 2 is mid, 3 is up
    @Override
    public Mat processFrame(Mat input){
        input.copyTo(output);//don't modify inputs ig

        contours.clear();//idk why we need it, but we have it so ok

        //we're using hsl i guess??
        hue = 53;//SET THIS VALUE!!!//53
        sensitivity = 30;//THIS ONE TOO!!!

        //step 1: blur
        Imgproc.GaussianBlur(input, blur, new Size(5,5), 0);//source, destination, size of blur ig, sigmax??? can be ommited

        //step 2: rgb to hsv... technically could do it first but ok
        Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_RGB2HSV);//source, dest, color swap choice (is an int technically)

        //find contours:
        Scalar lowBound = new Scalar((hue/2)-sensitivity,100,50);//basically what adrian did but in separate lines to make it look neat :)
        Scalar hiBound = new Scalar(hue+sensitivity,255,255);//also CHANGE THESE NUMS!!
        Core.inRange(hsv, lowBound, hiBound, singleColor);//source, low bound, high bound, destinatin
        Imgproc.findContours(singleColor, contours, hierarchy, Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_SIMPLE);//source, contours list, hierarchy mat, int for code, and int method
        //also test using RETR_TREE idk what the difference is oop

        if(contours.size() > 0){//in other words, it found stuff
            double max = 0;
            int maxInd = 0;
            for(int i = 0; i < contours.size(); i++){//loop through all the contours, and find the largest box. CHANGE THIS!!!! This might not be what we want!!!
                double area = Imgproc.contourArea(contours.get(i));
                if(area > max){
                    max = area;
                    maxInd = i;
                }
            }
            //draw a box of the largest one of single color
            Rect largestRect = Imgproc.boundingRect(contours.get(maxInd));
            Scalar boxColor = new Scalar(255, 255, 255);//should be white
            Imgproc.rectangle(output, largestRect, boxColor, 3, 8, 0);//Currently boxed based on rectangle, Change if needed!!!!

            //Determine which box it's in
            if (largestRect.x<240) {
                barcode=1;
                //telemetry.addData("Box", "1");
            }
            else if (largestRect.x<400) {
                barcode=2;
                //telemetry.addData("Box", "2");
            }
            else {
                barcode=3;
                //telemetry.addData("Box", "3");
            }

        }

        return output;
    }

    public int getCode() {
        return barcode;
    }
}
