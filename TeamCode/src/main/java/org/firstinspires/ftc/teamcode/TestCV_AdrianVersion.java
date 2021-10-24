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
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Testing based on Adrian example")
public class TestCV_AdrianVersion extends LinearOpMode {
    double hue;//color we want, yellow
    OpenCvCamera cam;// webcam
    int width;
    int height;
    Pipeline mainPipeline;
    double sensitivity;

    @Override
    public void runOpMode() throws InterruptedException{
        int camViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());//view for viewing what the webcam sees I think, on ds. Double check :grimacing:
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), camViewID);

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                mainPipeline = new Pipeline();//create new pipeline
                cam.setPipeline(mainPipeline);//set webcam pipeline

                width = 640;
                height = 480;

                cam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);//can add rotation if needed

                /*switch(mainPipeline.getCode()) {
                    case 0:
                        telemetry.addData("Box", "None");
                    case 1:
                        telemetry.addData("Box", "1");
                    case 2:
                        telemetry.addData("Box", "2");
                    case 3:
                        telemetry.addData("Box", "3");
                }
                telemetry.update();*/
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error...",":(");
                System.exit(0);
            }
        });



        waitForStart();

        //now start button is pressed, robot go!


    }

    class Pipeline extends OpenCvPipeline{
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
                telemetry.addData("Yellow?", "yes");
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
                    telemetry.addData("Box", "1");
                }
                else if (largestRect.x<400) {
                    barcode=2;
                    telemetry.addData("Box", "2");
                }
                else {
                    barcode=3;
                    telemetry.addData("Box", "3");
                }

            }
            else {
                telemetry.addData("Yellow?", "No");
            }

            telemetry.update();
            return output;
        }

        public int getCode() {
            return barcode;
        }
    }
}
