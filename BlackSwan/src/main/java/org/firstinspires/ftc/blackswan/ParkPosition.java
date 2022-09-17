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


    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new DeterminationPipeline(telemetry);
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

        while (opModeIsActive()) {

            telemetry.addData("Lightness", pipeline.AVG_L);
            telemetry.addData("GM", pipeline.AVG_A);
            telemetry.addData("BY", pipeline.AVG_B);

            telemetry.update();

            sleep(50);
        }
    }

    public static class DeterminationPipeline extends OpenCvPipeline {

        Telemetry telemetry;

        public DeterminationPipeline(Telemetry telemetry) {
            this.telemetry = telemetry;
        }

        public enum ParkingPos {
            left,
            right,
            middle,
            UNKNOWN
        }

        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar RED = new Scalar(255, 0, 0);

        static final Point DETECTION_ANCHOR = new Point(50, 50);

        static final int DETECTION_WIDTH = 30;
        static final int DETECTION_HEIGHT = 30;

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

        public volatile ParkingPos pos = ParkingPos.UNKNOWN;

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
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            pos = ParkingPos.UNKNOWN; // Record our analysis

            telemetry.update();

            // insert parking pos code here

            telemetry.update();

            return input;

        }

        public int getAnalysis1() {
            return AVG_L;
        }

        public int getAnalysis2() {
            return AVG_A;
        }

        public int getAnalysis3() {
            return AVG_B;
        }


    }
}
