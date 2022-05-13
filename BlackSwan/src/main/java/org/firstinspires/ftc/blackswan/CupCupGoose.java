package org.firstinspires.ftc.blackswan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "CupCupGoose.exe")
public class CupCupGoose extends LinearOpMode
{
    SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline(telemetry);
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("RED", pipeline.getAnalysis1());
            telemetry.addData("GREEN", pipeline.getAnalysis2());
            telemetry.addData("BLUE", pipeline.getAnalysis3());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        Telemetry telemetry;
        public SkystoneDeterminationPipeline(Telemetry telemetry){
            this.telemetry= telemetry;
        }

        /*
         * An enum to define the skystone position
         */
        public enum CupPosition
        {
            CUPLEFT,
            CUPMIDDLE,
            CUPRIGHT,
        }

        /*
              //outer blue 192, 176 size 30,42
   * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar RED = new Scalar(255, 0, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_BOTTOMLEFT_ANCHOR_POINT = new Point(1,240);
        static final Point REGION2_BOTTOMLEFT_ANCHOR_POINT = new Point(106,240);
        static final Point REGION3_BOTTOMLEFT_ANCHOR_POINT = new Point(213,240);



        static final int REGION_WIDTH = 106;
        static final int REGION_HEIGHT = 240;

//        final int FOUR_RING_THRESHOLD = 153;
//        final int THREE_RING_THRESHOLD = 149;
//        final int TWO_RING_THRESHOLD = 130;
//        final int ONE_RING_THRESHOLD = 119;

        Point region1_pointA = new Point(
                REGION1_BOTTOMLEFT_ANCHOR_POINT.x,
                REGION1_BOTTOMLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_BOTTOMLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_BOTTOMLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Point region2_pointA = new Point(
                REGION2_BOTTOMLEFT_ANCHOR_POINT.x,
                REGION2_BOTTOMLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_BOTTOMLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_BOTTOMLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Point region3_pointA = new Point(
                REGION3_BOTTOMLEFT_ANCHOR_POINT.x,
                REGION3_BOTTOMLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_BOTTOMLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_BOTTOMLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat region2_Cb;
        Mat region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;
        int avg2;
        int avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile CupPosition position = CupPosition.CUPLEFT;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            telemetry.addData("BooGaLoo", "1");
            telemetry.update();

            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            telemetry.addData("BooGaLoo", "2");
            telemetry.update();
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            telemetry.addData("BooGaLoo", "3");
            telemetry.update();
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
            telemetry.addData("BooGaLoo", "4");
            telemetry.update();

        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Negative thickness means solid fill

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Negative thickness means solid fill

            position = CupPosition.CUPLEFT; // Record our analysis
            telemetry.addData("value", avg1);
            telemetry.update();
//            if(avg1 > FOUR_RING_THRESHOLD) {
//                position = RingPosition.FOUR;
//            }else if (avg1 > THREE_RING_THRESHOLD) {
//                position = RingPosition.THREE;
//            }else if (avg1 > TWO_RING_THRESHOLD) {
//                position = RingPosition.TWO;
//            }else if (avg1 > ONE_RING_THRESHOLD){
//                position = RingPosition.ONE;
//            }else{
//                position = RingPosition.NONE;
//            }





            return input;
        }

        public int getAnalysis1()
        {
            return avg1;
        }

        public int getAnalysis2()
        {
            return avg2;
        }

        public int getAnalysis3()
        {
            return avg3;
        }


    }


}