package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "OpenCV Testing")

public class opencvanothertest extends LinearOpMode {

    double cX = 0;
    double cY = 0;
    double width = 0;
    YellowBlobDetectionPipeline pipeline;
    private OpenCvCamera controlHubCam;
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 360;
    public static final double objectWidthInRealWorldUnits = 3.75;
    public static final double focalLength = 728;


    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        controlHubCam = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        pipeline = new YellowBlobDetectionPipeline();
        controlHubCam.setPipeline(pipeline);

        controlHubCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                controlHubCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
                telemetry.update();
            }
        });

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch", (getDistance(width)));
            telemetry.addData("Yellow Distance (in)", pipeline.yellowDist);
            telemetry.addData("Blue Distance (in)", pipeline.blueDist);
            telemetry.addData("Red Distance (in)", pipeline.redDist);
            telemetry.addData("Center X", pipeline.cX);
            telemetry.addData("Center Y", pipeline.cY);
            telemetry.addData("Width px", pipeline.width);
            telemetry.update();

        }

        controlHubCam.stopStreaming();
    }


    class YellowBlobDetectionPipeline extends OpenCvPipeline {
        public double yellowDist = 0, blueDist = 0, redDist = 0;

        public double cX = 0.0;
        public double cY = 0.0;
        public double width = 0.0;

        private Mat hsv = new Mat();
        private Mat mask = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            detectColor(input, new Scalar(20, 100, 100), new Scalar(30, 255, 255), "Yellow");
            detectColor(input, new Scalar(100, 150, 0), new Scalar(140, 255, 255), "Blue");
//            detectRed(input);
            return input;
        }

        private void detectColor(Mat input, Scalar lower, Scalar upper, String colorName) {
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

            Mat mask = new Mat();
            Core.inRange(hsv, lower, upper, mask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

            Rect rect = Imgproc.boundingRect(mask);
            if (rect.width > 0 && rect.height > 0) {
                double widthLocal = rect.width;
                double dist = getDistance(widthLocal);
                dist = Math.round(dist * 100.0) / 100.0;

                Imgproc.putText(input, colorName + " Dist: " + dist + " in",
                        new Point(rect.x, rect.y - 5),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);

                if (colorName.equals("Yellow")) yellowDist = dist;
                if (colorName.equals("Blue")) blueDist = dist;
            }
        }

//        private void detectRed(Mat input) {
//            Mat hsv = new Mat();
//            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
//
//            // First red range
//            Mat mask1 = new Mat();
//            Core.inRange(hsv, new Scalar(0, 120, 70), new Scalar(10, 255, 255), mask1);
//
//            // Second red range
//            Mat mask2 = new Mat();
//            Core.inRange(hsv, new Scalar(170, 120, 70), new Scalar(180, 255, 255), mask2);
//
//            // Combine both
//            Mat redMask = new Mat();
//            Core.add(mask1, mask2, redMask);
//
//            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
//            Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_OPEN, kernel);
//            Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_CLOSE, kernel);
//
//            Rect rect = Imgproc.boundingRect(redMask);
//            if (rect.width > 0 && rect.height > 0) {
//                double widthLocal = rect.width;
//                double dist = getDistance(widthLocal);
//                dist = Math.round(dist * 100.0) / 100.0;
//
//                Imgproc.putText(input, "Red Dist: " + dist + " in",
//                        new Point(rect.x, rect.y - 5),
//                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);
//
//                redDist = dist;
//            }
//
//        }

    }
    private double getDistance(double widthPixels) {
        double focalLength = 728;
        double realWidthInches = 3.5;
        return (realWidthInches * focalLength) / widthPixels;
    }}
