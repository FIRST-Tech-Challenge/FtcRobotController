package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TestCV extends OpenCvPipeline {
    enum DuckLoc{
        L,
        C,
        R,
        NONE
    }

    private int width; //image width, do we need this?
    DuckLoc loc;//duck location

    public TestCV(int w){
        width = w;
    }

    @Override
    public Mat processFrame(Mat input){//mat=matrix, input=the image matrix coming from camera. returns a mat to draw on screen
        //determine how many duckies/dots camera can see, and determine from there? hmm we'll see

        Mat mat = new Mat();//return this mat
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);//changes input to hsv, put in mat ???

        //something went wrong, ig there's no duckie sooo
        if(mat.empty()){
            loc = DuckLoc.NONE;
            return input;
        }

        //find hsv range to detect duckie or the team marker
        Scalar lowHSV = new Scalar(2,3,4);//CHANGE THIS!!!, lower bound
        Scalar hiHSV = new Scalar(2,3,4);//CHANGE THIS!!!, upper bound
        //hue goes 0-179??? but whyyyy
        //sat and val both go from 0-255
        Mat thresh = new Mat();//a black and white, showing location of duckie
        Core.inRange(mat, lowHSV, hiHSV, thresh);//take the mat, if within range, then make white else black, store in thresh

        //detect edges??? is this necessary??
        Mat edges = new Mat();//this is a lot of mats....
        Imgproc.Canny(thresh, edges, 100,300);//adjust the 2 threshs, idek what they do sooooo

        //https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        //read that for more on bounding boxes
        //find bounding boxes
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();//idk if I like this...
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        //uhhh this is unfinished file I'm going to look at adrian's version now :grimacing:

        //here is source code for those that happen upon this file and want to read where I got this madness from:
        //https://gist.github.com/oakrc/12a7b5223df0cb55d7c1288ce96a6ab7

        //also note: this is a TESTING file so please don't mind my crazy comments
        //to be honest, they're rather informative
        //such as this one.
        //who reads comments?? hmmmm :thinking:
        return mat;
    }

}

/*
left: level 1, bottom
center: level 2, center
right: level 3, top
*/
