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
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "OpenCV Testing")

public class opencv1 extends LinearOpMode {

    // Public fields updated by pipeline for telemetry
    double cX_Y = 0, cY_Y = 0, width_Y = 0;
    double cX_R = 0, cY_R = 0, width_R = 0;
    double cX_B = 0, cY_B = 0, width_B = 0;

    private OpenCvCamera controlHubCam;
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 360;


    public static final double objectWidthInRealWorldUnits = 3.5;
    public static final double focalLength = 728;


    @Override
    public void runOpMode() {

        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Yellow:");
            telemetry.addData("Coordinate", "(" + (int) cX_Y + ", " + (int) cY_Y + ")");
            telemetry.addData("Distance (in)", getDistance(width_Y));

            telemetry.addLine("Red:");
            telemetry.addData("Coordinate", "(" + (int) cX_R + ", " + (int) cY_R + ")");
            telemetry.addData("Distance (in)", getDistance(width_R));

            telemetry.addLine("Blue:");
            telemetry.addData("Coordinate", "(" + (int) cX_B + ", " + (int) cY_B + ")");
            telemetry.addData("Distance (in)", getDistance(width_B));

            telemetry.update();
        }

        controlHubCam.stopStreaming();
    }

    private void initOpenCV() {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new opencv1.YellowBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

    class YellowBlobDetectionPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {

            Mat yellowMask = preprocessFrameY(input);
            Mat redMask = preprocessFrameR(input);
            Mat blueMask = preprocessFrameB(input);

            List<MatOfPoint> contoursY = new ArrayList<>();
            Imgproc.findContours(yellowMask, contoursY, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            List<MatOfPoint> contoursR = new ArrayList<>();
            Imgproc.findContours(redMask, contoursR, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            List<MatOfPoint> contoursB = new ArrayList<>();
            Imgproc.findContours(blueMask, contoursB, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);


            detectAndDraw(input, contoursY, new Scalar(0, 255, 255), "Yellow");
            detectAndDraw(input, contoursR, new Scalar(0, 0, 255), "Red");
            detectAndDraw(input, contoursB, new Scalar(255, 0, 0), "Blue");

            return input;
        }

        private void detectAndDraw(Mat input, List<MatOfPoint> contours, Scalar color, String label) {
            MatOfPoint largest = findLargestContour(contours);
            if (largest != null) {
                Rect rect = Imgproc.boundingRect(largest);
                double widthLocal = rect.width;
                Moments m = Imgproc.moments(largest);

                double cXLocal = 0, cYLocal = 0;
                if (m.get_m00() != 0) {
                    cXLocal = m.get_m10() / m.get_m00();
                    cYLocal = m.get_m01() / m.get_m00();
                }

                if (label.equals("Yellow")) { cX_Y = cXLocal; cY_Y = cYLocal; width_Y = widthLocal; }
                if (label.equals("Red")) { cX_R = cXLocal; cY_R = cYLocal; width_R = widthLocal; }
                if (label.equals("Blue")) { cX_B = cXLocal; cY_B = cYLocal; width_B = widthLocal; }

                Imgproc.drawContours(input, contours, contours.indexOf(largest), color, 2);
                Imgproc.circle(input, new Point(cXLocal, cYLocal), 5, color, -1);
                Imgproc.putText(input, label, new Point(cXLocal + 10, cYLocal - 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
                Imgproc.putText(input, "Width: " + (int) widthLocal + "px", new Point(cXLocal + 10, cYLocal), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);

                double dist = getDistance(widthLocal);
                dist = Math.round(dist * 100.0) / 100.0; // clean decimal
                Imgproc.putText(input, "Dist: " + dist + " in", new Point(cXLocal + 10, cYLocal + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
                Imgproc.putText(input, "(" + (int) cXLocal + ", " + (int) cYLocal + ")", new Point(cXLocal + 10, cYLocal + 40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
            }
        }

        private Mat preprocessFrameY(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerYellow = new Scalar(20, 100, 100);
            Scalar upperYellow = new Scalar(30, 255, 255);

            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

        private Mat preprocessFrameR(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);


            Scalar lowerRed1 = new Scalar(0, 120, 70);
            Scalar upperRed1 = new Scalar(10, 255, 255);
            Mat mask1 = new Mat();
            Core.inRange(hsvFrame, lowerRed1, upperRed1, mask1);


            Scalar lowerRed2 = new Scalar(170, 120, 70);
            Scalar upperRed2 = new Scalar(180, 255, 255);
            Mat mask2 = new Mat();
            Core.inRange(hsvFrame, lowerRed2, upperRed2, mask2);


            Mat redMask = new Mat();
            Core.add(mask1, mask2, redMask);


            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_CLOSE, kernel);

            return redMask;
        }


        private Mat preprocessFrameB(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerBlue = new Scalar(100, 150, 0);
            Scalar upperBlue = new Scalar(140, 255, 255);

            Mat blueMask = new Mat();
            Core.inRange(hsvFrame, lowerBlue, upperBlue, blueMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_CLOSE, kernel);

            return blueMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    private static double getDistance(double width) {
        if (width == 0) return 0; // prevent div-by-zero
        return (objectWidthInRealWorldUnits * focalLength) / width;
    }
}
