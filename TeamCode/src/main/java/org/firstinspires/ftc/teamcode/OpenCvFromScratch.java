package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *YES
 */
@Autonomous(name= "opencvSkystoneDetector", group="Sky autonomous")
//@Disabled//comment out this line before using
public class OpenCvFromScratch extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMidYellow = -1;
    private static int valLeftYellow = -1;
    private static int valRightYellow = -1;

    private static int valMidBlue = -1;
    private static int valLeftBlue = -1;
    private static int valRightBlue = -1;

    private static int valMidRed = -1;


    private static float rectHeight = 0.2f;
    private static float rectWidth = 0.2f;
    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera controlHubCam;
    static List<Mat> contoursListThing = new ArrayList<>();


    @Override
    public void runOpMode() throws InterruptedException {
        contoursListThing.add(new Mat());
        contoursListThing.add(new Mat());
        contoursListThing.add(new Mat());

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new StageSwitchingPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);


        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            /*telemetry.addData("Yellow", valLeftYellow+"   "+valMidYellow+"   "+valRightYellow);
            telemetry.addData("Blue", valLeftBlue+"   "+valMidBlue+"   "+valRightBlue);
            telemetry.addData("Red", 0+"   "+valMidRed+"   "+0);*/
            telemetry.addData("Yellow", valMidYellow);
            telemetry.addData("Blue", valMidBlue);
            telemetry.addData("Red", valMidRed);


            telemetry.addData("matListSize", contoursListThing.size());
            telemetry.addData("matList", contoursListThing);

            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();
            sleep(100);
            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);

        }
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2MatBlue = new Mat();
        Mat yCbCrChan2MatRed = new Mat();

        Mat thresholdMatYellow = new Mat();
        Mat thresholdMatBlue = new Mat();
        Mat thresholdMatRed = new Mat();

        Mat all = new Mat();
        List<MatOfPoint> contoursListYellow = new ArrayList<>();
        List<MatOfPoint> contoursListBlue = new ArrayList<>();
        List<MatOfPoint> contoursListRed = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursListYellow.clear();
            contoursListBlue.clear();
            contoursListRed.clear();

            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            //Core.split(input, contoursListThing);

            Imgproc.cvtColor(input, yCbCrChan2MatBlue, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Imgproc.cvtColor(input, yCbCrChan2MatRed, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb

            Core.extractChannel(yCbCrChan2MatBlue, yCbCrChan2MatBlue, 2);//takes cb difference and stores
            Core.extractChannel(yCbCrChan2MatRed, yCbCrChan2MatRed, 1);//takes cb difference and stores

            //Core.mixChannels();

            //b&w
            Imgproc.threshold(yCbCrChan2MatBlue, thresholdMatYellow, 60, 255, Imgproc.THRESH_BINARY_INV);
            Imgproc.threshold(yCbCrChan2MatBlue, thresholdMatBlue, 190, 255, Imgproc.THRESH_BINARY);


            //contoursListThing = new ArrayList<>();



            //Core.extractChannel(yCbCrChan2MatRed, yCbCrChan2MatRed, 1);

            Imgproc.threshold(yCbCrChan2MatRed, thresholdMatRed, 190, 255, Imgproc.THRESH_BINARY);

            //102
            //y 60
            //r 160
            //b 180
            //outline/contour
            Imgproc.findContours(thresholdMatYellow, contoursListYellow, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(thresholdMatBlue, contoursListBlue, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(thresholdMatRed, contoursListRed, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            yCbCrChan2MatBlue.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMidRed = thresholdMatRed.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMidRed = (int)pixMidRed[0];

            double[] pixMidYellow = thresholdMatYellow.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMidYellow = (int)pixMidYellow[0];

            double[] pixMidBlue = thresholdMatBlue.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMidBlue = (int)pixMidBlue[0];

            /*double[] pixLeftYellow = thresholdMatYellow.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeftYellow = (int)pixLeftYellow[0];

            double[] pixLeftBlue = thresholdMatBlue.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeftBlue = (int)pixLeftBlue[0];

            double[] pixRightYellow = thresholdMatYellow.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRightYellow = (int)pixRightYellow[0];

            double[] pixRightBlue = thresholdMatBlue.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRightBlue = (int)pixRightBlue[0];*/

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            /*Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);*/

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMatYellow;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}