package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@TeleOp(name = "ShapeDetection", group = "Vision")
public class opencvShapes extends LinearOpMode {

    private OpenCvWebcam webcam;
    private ShapeDetectionPipeline pipeline;

    @Override
    public void runOpMode() {
        final String webcamName = "Webcam 1";
        final int streamWidth = 320;
        final int streamHeight = 240;
        final OpenCvCameraRotation rotation = OpenCvCameraRotation.UPRIGHT;

        WebcamName cam;
        try {
            cam = hardwareMap.get(WebcamName.class, webcamName);
        } catch (Exception e) {
            telemetry.addData("ERROR", "Webcam '%s' not found", webcamName);
            telemetry.update();
            return;
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(cam, cameraMonitorViewId);
        pipeline = new ShapeDetectionPipeline();
        webcam.setPipeline(pipeline);

        telemetry.addData("Status", "Opening camera...");
        telemetry.update();

        final AtomicReference<Boolean> cameraOpened = new AtomicReference<>(false);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(streamWidth, streamHeight, rotation);
                cameraOpened.set(true);
                telemetry.addData("Camera", "Opened and streaming");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera", "Error opening: " + errorCode);
                telemetry.update();
            }
        });

        FtcDashboard.getInstance().startCameraStream(webcam, 30);

        try {
            waitForStart();
            while (opModeIsActive()) {
                ShapeDetectionPipeline.ShapeSnapshot snap = pipeline.getLatest();

                telemetry.addData("Circles", snap.circleCount);
                telemetry.addData("Rectangles", snap.rectangleCount);
                telemetry.addData("Hexagons", snap.hexagonCount);
                telemetry.addData("Triangles", snap.triangleCount);
                telemetry.update();

                sleep(50);
            }
        } finally {
            if (webcam != null) {
                try { webcam.stopStreaming(); } catch (RuntimeException ignored) {}
                try { webcam.closeCameraDevice(); } catch (RuntimeException ignored) {}
            }
        }
    }

    private static class ShapeDetectionPipeline extends OpenCvPipeline {
        private final Mat gray = new Mat();
        private final Mat blur = new Mat();
        private final Mat edges = new Mat();
        private final AtomicReference<ShapeSnapshot> latest = new AtomicReference<>(new ShapeSnapshot(0, 0, 0,0));

        public static class ShapeSnapshot {
            public final int triangleCount;
            public final int circleCount;
            public final int rectangleCount;
            public final int hexagonCount;
            public ShapeSnapshot(int circles, int rects, int hexes, int tris) {
                this.circleCount = circles;
                this.rectangleCount = rects;
                this.hexagonCount = hexes;
                this.triangleCount = tris;
            }
        }

        public ShapeSnapshot getLatest() {
            return latest.get();
        }

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGBA2GRAY);
            Imgproc.GaussianBlur(gray, blur, new Size(5, 5), 0);

            int circleCount = detectCircles(input);
            int rectCount = 0;
            int hexCount = 0;
            int triCount = 0;

            // Edge detection for polygon finding
            Imgproc.Canny(blur, edges, 50, 150);
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                double peri = Imgproc.arcLength(contour2f, true);
                MatOfPoint2f approx = new MatOfPoint2f();
                Imgproc.approxPolyDP(contour2f, approx, 0.04 * peri, true);

                int vertices = (int) approx.total();
                if (vertices == 3) {
                    triCount++;
                }
                if (vertices == 4) {
                    rectCount++;
                    Imgproc.drawContours(input, List.of(new MatOfPoint(approx.toArray())), -1, new Scalar(255, 0, 0), 2);
                }
                if (vertices == 6) {
                    hexCount++;
                    Imgproc.drawContours(input, List.of(new MatOfPoint(approx.toArray())), -1, new Scalar(0, 255, 0), 2);
                }

                contour2f.release();
                approx.release();
            }

            hierarchy.release();
            for (MatOfPoint contour : contours) contour.release();

            latest.set(new ShapeSnapshot(circleCount, rectCount, hexCount, triCount));
            return input;
        }

        private int detectCircles(Mat input) {
            Mat circles = new Mat();
            Imgproc.HoughCircles(blur, circles, Imgproc.HOUGH_GRADIENT, 1.2, 20, 100, 30, 10, 100);
            int count = circles.cols();
            for (int i = 0; i < count; i++) {
                double[] data = circles.get(0, i);
                if (data != null) {
                    Point center = new Point(data[0], data[1]);
                    int radius = (int) Math.round(data[2]);
                    Imgproc.circle(input, center, radius, new Scalar(0, 0, 255), 2);
                    Imgproc.circle(input, center, 2, new Scalar(0, 255, 255), 3);
                }
            }
            circles.release();
            return count;
        }
    }
}

