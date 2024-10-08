package org.firstinspires.ftc.teamcode.configs;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


@TeleOp
public class Config_DetectColor extends LinearOpMode {

    //Webcam variables
    OpenCvCamera webcam; //Represents the webcam


    @Override
    public void runOpMode() {
        // Initialize the pipeline
        ColorPipelineTest pipeline = new ColorPipelineTest();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam.setPipeline(pipeline);



        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //Start streaming with anresolution of 640x480 and upright rotation
                //Adjust resolution as needed; higher resolutions require more processing power
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode){
                //Handle camera opening errors
                telemetry.addData("Camera Error", "Failed to open camera with error code:"+errorCode);
                telemetry.update();
            }
        });

        boolean end = false;
        int lowerBound = 0;
        int upperBound = 10;
        Scalar newLowerTest = new Scalar(lowerBound, 30, 30);
        Scalar newUpperTest = new Scalar(upperBound, 255, 255);
        while(!end)
        {
            pipeline.lowerTest = newLowerTest;
            pipeline.upperTest = newUpperTest;
            if(gamepad1.dpad_up) {
                lowerBound += 10;
                upperBound += 10;
                telemetry.addData("Lower Bound:",lowerBound);
                telemetry.addData("Upper Bound:",upperBound);
                telemetry.update();
                newLowerTest = new Scalar(lowerBound, 30, 30);
                newUpperTest = new Scalar(upperBound, 255, 255);
                sleep(100);
            }
            if(gamepad1.dpad_down)
            {
                lowerBound -= 10;
                upperBound -= 10;
                telemetry.addData("Lower Bound:",lowerBound);
                telemetry.addData("Upper Bound:",upperBound);
                telemetry.update();
                newLowerTest = new Scalar(lowerBound, 30, 30);
                newUpperTest = new Scalar(upperBound, 255, 255);
                sleep(100);
            }
            if(gamepad1.triangle)
            {
                end = true;
            }

        }



        //wait for the game to start (driver presses PLAY)
        waitForStart();

        //Main loop
        while (opModeIsActive()) {

            telemetry.update();
            sleep(50);


        }

        //Stop streaming when OpMode is no longer active
        webcam.stopStreaming();
    }


    private class ColorPipelineTest extends OpenCvPipeline {
        // Define color ranges in HSV (Hue, Saturation, Value)
        // HSV ranges: Hue (0-179), Saturation (0-255), Value (0-255)

        //Yellow color range
        private final Scalar lowerYellow = new Scalar(20, 50, 70);
        private final Scalar upperYellow = new Scalar(40, 255, 255);



        //Blue color range
        public Scalar lowerBlue = new Scalar(105, 50, 70);
        public Scalar upperBlue = new Scalar(135, 255, 255);

        //Red color range (note: red wraps around the hue spectrum, so we use two ranges)
        public Scalar lowerRed1 = new Scalar(0, 50, 70);
        public Scalar upperRed1 = new Scalar(9, 255, 255);
        public Scalar lowerRed2 = new Scalar(160, 50, 70);
        public Scalar upperRed2 = new Scalar(179, 255, 255);

        public Scalar lowerTest = new Scalar(0, 30, 30);
        public Scalar upperTest = new Scalar(10, 255, 255);

        /**
         * Datected color
         * <p>
         * This enum represents the possible colors that can be detected.
         */
        private Mat hsv = new Mat();
        @Override
        public Mat processFrame(Mat input) {
            //Convert the imput image from BGR to HSV color space

            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

            //Create binary masks for each color based on defined HSV ranges
            //Mat maskBlue = new Mat();
            Core.inRange(hsv, lowerTest, upperTest, hsv);
            return(hsv);
            //return maskBlue;
        }



        private double calculateArea(Mat mask) {
            return Math.round(Core.sumElems(mask).val[0] / 100) / 100;

        }



    }
}
