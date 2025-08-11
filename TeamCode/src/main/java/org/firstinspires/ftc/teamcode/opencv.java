// SO PERCENT 2F SUCKS yeah color label kinda
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

public class opencv extends LinearOpMode {

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

        // HSV bounds for colors
        private final Scalar lowerYellow = new Scalar(20, 100, 100);
        private final Scalar upperYellow = new Scalar(30, 255, 255);

        private final Scalar lowerBlue = new Scalar(100, 150, 0);
        private final Scalar upperBlue = new Scalar(140, 255, 255);

        private final Scalar lowerRed1 = new Scalar(0, 100, 100);
        private final Scalar upperRed1 = new Scalar(10, 255, 255);
        private final Scalar lowerRed2 = new Scalar(160, 100, 100);
        private final Scalar upperRed2 = new Scalar(179, 255, 255);

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            boolean yellowDetected = detectColor(hsv, lowerYellow, upperYellow, "Yellow");
            boolean blueDetected   = detectColor(hsv, lowerBlue, upperBlue, "Blue");
            boolean redDetected    = detectRed(hsv, lowerRed1, upperRed1, lowerRed2, upperRed2);

            // Draw text
            if (yellowDetected) Imgproc.putText(input, "Yellow", new Point(30, 30), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 255, 0), 2);
            if (blueDetected)   Imgproc.putText(input, "Blue", new Point(30, 60), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 0, 0), 2);
            if (redDetected)    Imgproc.putText(input, "Red", new Point(30, 90), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 0, 255), 2);

            hsv.release();
            return input;
        }

        // Detect Yellow or Blue
        private boolean detectColor(Mat hsv, Scalar lower, Scalar upper, String colorName) {
            Mat mask = new Mat();
            Core.inRange(hsv, lower, upper, mask);

            Mat hierarchy = new Mat();
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            boolean detected = false;

            if (!contours.isEmpty()) {
                double maxArea = 0;
                MatOfPoint largestContour = null;
                for (MatOfPoint contour : contours) {
                    double area = Imgproc.contourArea(contour);
                    if (area > maxArea) {
                        maxArea = area;
                        largestContour = contour;
                    }
                }

                if (largestContour != null) {
                    Rect rect = Imgproc.boundingRect(largestContour);
                    Imgproc.rectangle(hsv, rect, new Scalar(0, 255, 0), 2);

                    // Update pipeline fields
                    cX = rect.x + rect.width / 2.0;
                    cY = rect.y + rect.height / 2.0;
                    width = rect.width;

                    detected = true;
                }
            }

            mask.release();
            hierarchy.release();
            return detected;
        }

        // Detect Red (needs two ranges for hue wraparound)
        private boolean detectRed(Mat hsv, Scalar lower1, Scalar upper1, Scalar lower2, Scalar upper2) {
            Mat mask1 = new Mat();
            Mat mask2 = new Mat();
            Core.inRange(hsv, lower1, upper1, mask1);
            Core.inRange(hsv, lower2, upper2, mask2);

            Mat mask = new Mat();
            Core.add(mask1, mask2, mask);

            Mat hierarchy = new Mat();
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            boolean detected = false;

            if (!contours.isEmpty()) {
                double maxArea = 0;
                MatOfPoint largestContour = null;
                for (MatOfPoint contour : contours) {
                    double area = Imgproc.contourArea(contour);
                    if (area > maxArea) {
                        maxArea = area;
                        largestContour = contour;
                    }
                }

                if (largestContour != null) {
                    Rect rect = Imgproc.boundingRect(largestContour);
                    Imgproc.rectangle(hsv, rect, new Scalar(0, 255, 0), 2);

                    // Update pipeline fields
                    cX = rect.x + rect.width / 2.0;
                    cY = rect.y + rect.height / 2.0;
                    width = rect.width;

                    detected = true;
                }
            }

            mask1.release();
            mask2.release();
            mask.release();
            hierarchy.release();
            return detected;
        }

    }
    private double getDistance(double widthPixels) {
        double focalLength = 728;
        double realWidthInches = 3.5;
        return (realWidthInches * focalLength) / widthPixels;
    }}