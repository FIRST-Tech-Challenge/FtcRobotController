package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Testing based on Adrian example")
public class TestCV_AdrianVersion extends LinearOpMode {
    double hue;
    OpenCvCamera cam;
    int width;
    int height;
    Pipeline mainPipeline;//Create Class First!!! Change Name!!!
    double sensitivity;

    @Override
    public void runOpMode() throws InterruptedException{
        int camViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());//view for viewing what the webcam sees I think, on ds. Double check :grimacing:
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), camViewID);

        cam.openCameraDevice();//start the webcam

        mainPipeline = new Pipeline();//create new pipeline
        cam.setPipeline(mainPipeline);//set webcam pipeline

        width = 640;
        height = 480;

        cam.startStreaming(width, height);//can add rotation if needed

        waitForStart();

        //now start button is pressed, robot go!

        int barcode = 0; //level 1 is bottom, level 2 is mid, 3 is up

        //idk how we're measuring the duckies yet but this is where we set target zones!


    }

    class Pipeline extends OpenCvPipeline{
        List<MatOfPoint> contours = new ArrayList<>();

        //WHY DO WE NEED SO MANY MATS????!?!?!?!
        Mat hsv = new Mat();
        Mat blur = new Mat();
        Mat output = new Mat();
        Mat singleColor = new Mat();
        Mat hierarchy = new Mat();

        @Override
        public Mat processFrame(Mat input){
            input.copyTo(output);//don't modify inputs ig

            contours.clear();//idk why we need it, but we have it so ok

            //we're using hsl i guess??
            hue = 2;//SET THIS VALUE!!!
            sensitivity = 2;//THIS ONE TOO!!!

            //step 1: blur
            Imgproc.GaussianBlur(input, blur, new Size(5,5), 0);//source, destination, size of blur ig, sigmax??? can be ommited

            //step 2: bgr to hsv... technically could do it first but ok
            Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_RGB2HSV);//source, dest, color swap choice (is an int technically)

            //find contours:
            Scalar lowBound = new Scalar(2,3,4);//basically what adrian did but in separate lines to make it look neat :)
            Scalar hiBound = new Scalar(2,3,4);//also CHANGE THESE NUMS!!
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
                Point lowerLeft = new Point(0,0);//Change This!!!! Will change with game!!!
                Point upperRight = new Point(30,30);//This Too!!!
                Scalar boxColor = new Scalar(255, 255, 255);//should be white
                Imgproc.rectangle(output, largestRect, boxColor, -1, 8, 0);//Currently boxed based on rectangle, Change if needed!!!!

            }

            return output;
        }
    }
}
