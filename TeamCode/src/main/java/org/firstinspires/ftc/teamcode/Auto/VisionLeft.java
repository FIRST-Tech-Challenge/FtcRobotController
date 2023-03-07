package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisionLeft {
    OpMode opMode;
    private OpenCvCamera camera_red;

    AddBoxesPipeline pipeline;

    private final Point centerBox_topLeft    = new Point(185,75);
    private final Point centerBox_bottomRight    = new Point(230, 140);

//    private final Point centerBox_topLeft    = new Point(230,30);
//    private final Point centerBox_bottomRight    = new Point(190, 100);



    Mat YCrCb = new Mat();
    Mat red = new Mat();
    Mat green = new Mat();
    Mat blue = new Mat();
    Mat region_center_red, region_center_green, region_center_blue;
    int red_avg, green_avg, blue_avg;
    boolean one = false, two = false, three = false;


    public VisionLeft(OpMode op){

        opMode = op;

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        camera_red = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam Red"), cameraMonitorViewId);

        pipeline = new AddBoxesPipeline();
        camera_red.openCameraDevice();
        camera_red.setPipeline(pipeline);
        camera_red.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

    }

    public void stopStreaming(){
        camera_red.stopStreaming();
    }

    //Add boxes to the image display
    class AddBoxesPipeline extends OpenCvPipeline {

        void inputToColors(Mat input)
        {
            //Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(input, red, 0);
            Core.extractChannel(input, green, 1);
            Core.extractChannel(input, blue, 2);

        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToColors(firstFrame);

            region_center_red = red.submat(new Rect(centerBox_topLeft,centerBox_bottomRight));
            region_center_green = green.submat(new Rect(centerBox_topLeft,centerBox_bottomRight));
            region_center_blue = blue.submat(new Rect(centerBox_topLeft,centerBox_bottomRight));

        }

        //This processes the visual output on the screen
        @Override
        public Mat processFrame(Mat input){

            inputToColors(input);


            red_avg = (int) Core.mean(region_center_red).val[0];
            green_avg = (int) Core.mean(region_center_green).val[0];
            blue_avg = (int) Core.mean(region_center_blue).val[0];

//            opMode.telemetry.addData("RED: ", red_avg);
//            opMode.telemetry.addData("GREEN: ", green_avg);
//            opMode.telemetry.addData("BLUE: ", blue_avg);
//            opMode.telemetry.update();

            if(red_avg >= 110){
                one = true;
                two = false;
                three = false;
            }

            else if(blue_avg >= 110){
                one = false;
                two = false;
                three = true;
            }

            //opMode.telemetry.addData("boxRight: ", right_avg);
            //opMode.telemetry.update();
//            else if(right_avg <= left_avg && right_avg <= center_avg){
            else{
                one = false;
                two = true;
                three = false;
            }

            int thickness = 3;
            Scalar red = new Scalar(0,0,0);
            Scalar green = new Scalar(0,0,0);
            Scalar blue = new Scalar(0,0,0);

            if (one){
                red = new Scalar(255,0,0);
                Imgproc.rectangle(input, centerBox_topLeft, centerBox_bottomRight, red, thickness);
            }
            else if (two){
                green = new Scalar(0,255,0);
                Imgproc.rectangle(input, centerBox_topLeft, centerBox_bottomRight, green, thickness);
            } else {
                blue = new Scalar(0,0,255);
                Imgproc.rectangle(input, centerBox_topLeft, centerBox_bottomRight, blue, thickness);
            }

            return input;
        }

    }
}