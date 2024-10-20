package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;

public class TgeDetectorPipeline extends OpenCvPipeline {



    Rect z1Rect = new Rect(105, 210, 65, 65);
    Rect z2Rect = new Rect(240, 200, 65, 65);
    Rect z3Rect = new Rect(375  , 205, 65, 65);
    Scalar red = new Scalar(255, 0, 0);
    Scalar blue = new Scalar(0, 0, 255);

    Mat z1, z2, z3;

    Scalar tgeColor;
    int tgeZone = -1;
    int defaultZone = 1;
    double THRESHOLD = 180;
    Scalar z1AvgColor, z2AvgColor, z3AvgColor;

    public TgeDetectorPipeline(String color) {
        super();
        if (color.equals("blue")) {
            tgeColor = blue; // Blue
        } else {
            tgeColor = red; // Red
        }
    }

    @Override
    public Mat processFrame(Mat frame) {
        double dist[] = new double[3];
        //Scalar z1AvgColor, z2AvgColor, z3AvgColor;

        z1 = frame.submat(z1Rect);
        z2 = frame.submat(z2Rect);
        z3 = frame.submat(z3Rect);

        z1AvgColor = Core.mean(z1);
        z2AvgColor = Core.mean(z2);
        z3AvgColor = Core.mean(z3);

        //z1.release();
        //z2.release();
        //z3.release();

        dist[0] = colorDist(z1AvgColor, tgeColor);
        dist[1] = colorDist(z2AvgColor, tgeColor);
        dist[2] = colorDist(z3AvgColor, tgeColor);

        int min = 0;
        for (int i = 1; i < 3; i++) {
            if (dist[i] < dist[min]) {
                min = i;
            }
        }
        /*
        if (dist[min] < THRESHOLD) {
            tgeZone = defaultZone;
        } else {
            tgeZone = min + 1;
        }
         */
        tgeZone = min + 1;
        return frame;
    }

    public double colorDist(Scalar c1, Scalar c2){
       // double rDiff = c1.val[0] - c2.val[0];
        //double gDiff = c1.val[1] - c2.val[1];
        //double bDiff = c1.val[2] - c2.val[2];
        //return Math.sqrt(rDiff * rDiff + gDiff * gDiff + bDiff * bDiff);
        double dR = c1.val[0]-c2.val[0];
        double dG = c1.val[1]-c2.val[1];
        double dB = c1.val[2]-c2.val[2];
        double R = (c1.val[0] + c2.val[0])/2;
        if(R < 128){
            return Math.sqrt(
                    2*dR*dR + 4*dG*dG + 3*dB*dB
            );
        }
        return Math.sqrt(
                3*dR*dR + 4*dG*dG + 2*dB*dB
        );
    }

    public int getTgeZone() {
        return tgeZone;
    }
    public void setTgeColor(String color) {
        if (color.equals("blue")) {
            tgeColor = blue; // Blue
        } else {
            tgeColor = red; // Red
        }
    }
    public Scalar getZ1() {
        return z1AvgColor;
    }
    public Scalar getZ2() {
        return z2AvgColor;
    }
    public Scalar getZ3() {
        return z3AvgColor;
    }
}
