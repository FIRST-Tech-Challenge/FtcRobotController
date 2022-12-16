package org.firstinspires.ftc.blackswan;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

@Autonomous(name = "ParkingAutonomous")
public class ParkingAutonomous extends LinearOpMode {

    Robot robot;
    DeterminationPipeline pipeline;
    TelemetryPacket packet = new TelemetryPacket();
    public static int ParkDot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, this);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        pipeline = new DeterminationPipeline(telemetry, packet);
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

        if (ParkDot == 1){
            robot.forward(2.2,.5);
            robot.left(2.2,.5);
        } else if (ParkDot == 2){
            robot.forward(2.2,.5);
        }else if (ParkDot == 3){
            robot.forward(2.2,.5);
            robot.right(2.2, .5);
        } else {
            robot.forward(2.2, .5);
        }
        while (opModeIsActive()) {

            telemetry.addData("Lightness", pipeline.AVG_L);
            telemetry.addData("GM", pipeline.AVG_A);
            telemetry.addData("BY", pipeline.AVG_B);

            FtcDashboard dashboard = FtcDashboard.getInstance();
            packet.put("Lightness", pipeline.AVG_L);
            packet.put("GM", pipeline.AVG_A);
            packet.put("BY", pipeline.AVG_B);
            dashboard.sendTelemetryPacket(packet);


            telemetry.update();

            sleep(50);
        }
    }

    public static class DeterminationPipeline extends OpenCvPipeline {

        Telemetry telemetry;
        TelemetryPacket packet;

        public DeterminationPipeline(Telemetry telemetry, TelemetryPacket packet) {
            this.telemetry = telemetry;
            this.packet = packet;
        }

        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar RED = new Scalar(255, 0, 0);

        static final Point DETECTION_ANCHOR = new Point(200, 60);

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
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            telemetry.update();



            // A = GM // B = BY
            // one dot values, GM 174, BY 124
            // two dot values, GM 116, BY 179
            // three dot values, GM 127, BY 87

            if((AVG_A > 168) && (AVG_A < 208) && (AVG_B > 120) && (AVG_B < 160)){
                telemetry.addData("ONE DOT", "Current park position");
                ParkDot = 1;
            } else if((AVG_A > 100) && (AVG_A < 140) && (AVG_B > 170) && (AVG_B < 210)){
                telemetry.addData("TWO DOT", "Current park position");
                ParkDot = 2;
            } else if((AVG_A > 103) && (AVG_A < 143) && (AVG_B > 80) && (AVG_B < 120)){
                telemetry.addData("THREE DOT", "Current park position");
                ParkDot = 3;
            } else {
                telemetry.addData("NO DOT", "Current park position");
            }

            telemetry.update();

            FtcDashboard dashboard = FtcDashboard.getInstance();
            packet.put("Position", ParkDot);
            dashboard.sendTelemetryPacket(packet);

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
