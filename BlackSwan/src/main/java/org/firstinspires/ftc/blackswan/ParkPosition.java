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
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "AutonomousParkingTest")
public class ParkPosition extends LinearOpMode {

    DeterminationPipeline pipeline;
    public static int position;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);

        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();
        telemetry.addData("|Parking Position|>", position);
    }

    public static class DeterminationPipeline extends OpenCvPipeline {

        Telemetry telemetry;

        public DeterminationPipeline(Telemetry telemetry) {
            this.telemetry = telemetry;
        }

        static final Scalar RED = new Scalar(255, 0, 0);

        static final Point DETECTION_ANCHOR = new Point(140, 70);

        static final int DETECTION_WIDTH = 20;
        static final int DETECTION_HEIGHT = 20;

        Point region1_pointA = new Point(
                DETECTION_ANCHOR.x,
                DETECTION_ANCHOR.y);
        Point region1_pointB = new Point(
                DETECTION_ANCHOR.x + DETECTION_WIDTH,
                DETECTION_ANCHOR.y - DETECTION_HEIGHT);

        Mat DETECTION_L;
        Mat DETECTION_A;
        Mat DETECTION_B;

        Mat LAB = new Mat();

        Mat l = new Mat();
        Mat a = new Mat();
        Mat b = new Mat();

        int AVG_L;
        int AVG_A;
        int AVG_B;

        void inputToLAB(Mat input) {

            Imgproc.cvtColor(input, LAB, Imgproc.COLOR_RGB2Lab);

            Core.extractChannel(LAB, l, 0);
            Core.extractChannel(LAB, a, 1);
            Core.extractChannel(LAB, b, 2);
        }

        public void init(Mat firstFrame) {
            inputToLAB(firstFrame);

            DETECTION_L = l.submat(new Rect(region1_pointA, region1_pointB));
            DETECTION_A = a.submat(new Rect(region1_pointA, region1_pointB));
            DETECTION_B = b.submat(new Rect(region1_pointA, region1_pointB));

        }

        @Override
        public Mat processFrame(Mat input) {

            inputToLAB(input);

            DETECTION_L = l.submat(new Rect(region1_pointA, region1_pointB));
            DETECTION_A = a.submat(new Rect(region1_pointA, region1_pointB));
            DETECTION_B = b.submat(new Rect(region1_pointA, region1_pointB));

            AVG_L = (int) Core.mean(DETECTION_L).val[0];
            AVG_A = (int) Core.mean(DETECTION_A).val[0];
            AVG_B = (int) Core.mean(DETECTION_B).val[0];

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    RED,
                    2);

            if((AVG_A > 168) && (AVG_A < 208) && (AVG_B > 120) && (AVG_B < 160)){
                position=1;
            } else if((AVG_A > 100) && (AVG_A < 140) && (AVG_B > 170) && (AVG_B < 210)){
                position=2;
            } else if((AVG_A > 103) && (AVG_A < 143) && (AVG_B > 80) && (AVG_B < 120)){
                position=3;
            }

            return input;

        }
    }
}