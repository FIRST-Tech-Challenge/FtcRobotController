package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@TeleOp(name = "opencvFINAL", group = "Vision")
public class opencvFINAL extends LinearOpMode {

    private OpenCvWebcam webcam;
    private ColorDetectionPipeline pipeline;

    @Override
    public void runOpMode() {
        final String webcamName = "Webcam 1";      // <-- set to your webcam config name
        final int streamWidth = 320;               // change to 640 if you need more detail and the hub/phone can handle it
        final int streamHeight = 240;
        final OpenCvCameraRotation rotation = OpenCvCameraRotation.UPRIGHT;

        // safe webcam get
        WebcamName cam;
        try {
            cam = hardwareMap.get(WebcamName.class, webcamName);
        } catch (Exception e) {
            telemetry.addData("ERROR", "Webcam '%s' not found in hardwareMap", webcamName);
            telemetry.update();
            return;
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(cam, cameraMonitorViewId);

        pipeline = new ColorDetectionPipeline();
        webcam.setPipeline(pipeline);

        telemetry.addData("Status", "Opening camera...");
        telemetry.update();

        final AtomicReference<Boolean> cameraOpened = new AtomicReference<>(false);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                webcam.startStreaming(streamWidth, streamHeight, rotation);
                cameraOpened.set(true);
                telemetry.addData("Camera", "Opened and streaming");
                telemetry.update();
            }
            @Override public void onError(int errorCode) {
                telemetry.addData("Camera", "Error opening camera: " + errorCode);
                telemetry.update();
            }
        });
        FtcDashboard.getInstance().startCameraStream(webcam, 30);
        // MAIN OP MODE: ensure camera and mats are cleaned up even if opMode ends
        try {
            waitForStart();

            // main loop
            while (opModeIsActive()) {
                // read an atomic snapshot from pipeline
                ColorDetectionPipeline.DetectionSnapshot snap = pipeline.getLatest();

                // Yellow
                if (snap.yellowRect != null) {
                    telemetry.addData("Yellow", "YES area=%.0f", snap.yellowRect.area());
                    telemetry.addData("Y center", "(%.1f, %.1f)", snap.yellowCenter.x, snap.yellowCenter.y);
                } else {
                    telemetry.addData("Yellow", "no");
                }

                // Blue
                if (snap.blueRect != null) {
                    telemetry.addData("Blue", "YES area=%.0f", snap.blueRect.area());
                    telemetry.addData("B center", "(%.1f, %.1f)", snap.blueCenter.x, snap.blueCenter.y);
                } else {
                    telemetry.addData("Blue", "no");
                }

                // Red
                if (snap.redRect != null) {
                    telemetry.addData("Red", "YES area=%.0f", snap.redRect.area());
                    telemetry.addData("R center", "(%.1f, %.1f)", snap.redCenter.x, snap.redCenter.y);
                } else {
                    telemetry.addData("Red", "no");
                }

                telemetry.update();

                sleep(50);
            }
        } finally {
            // cleanup: stop streaming and close camera
            if (webcam != null) {
                try {
                    webcam.stopStreaming();
                } catch (RuntimeException ignored) {}
                try {
                    webcam.closeCameraDevice();
                } catch (RuntimeException ignored) {}
            }
            // release pipeline mats
            if (pipeline != null) {
                pipeline.releaseResources();
            }
        }
    }

    /**
     * ColorDetectionPipeline:
     * - detects yellow, blue, red (red uses two hue ranges)
     * - publishes an immutable DetectionSnapshot via AtomicReference
     * - reuses Mats to avoid allocations each frame
     */
    private static class ColorDetectionPipeline extends OpenCvPipeline {

        // ----- tunable HSV bounds (H: 0..180, S:0..255, V:0..255) -----
        private final Scalar Y_LOWER = new Scalar(18, 100, 100);
        private final Scalar Y_UPPER = new Scalar(35, 255, 255);

        private final Scalar B_LOWER = new Scalar(95, 120, 70);
        private final Scalar B_UPPER = new Scalar(140, 255, 255);

        // red wraps around hue boundary -> two ranges
        private final Scalar R1_LOWER = new Scalar(0, 120, 70);
        private final Scalar R1_UPPER = new Scalar(10, 255, 255);
        private final Scalar R2_LOWER = new Scalar(170, 120, 70);
        private final Scalar R2_UPPER = new Scalar(180, 255, 255);

        private final double MIN_CONTOUR_AREA = 900; // tune this for your camera/resolution

        // Mats reused to reduce allocations
        private final Mat hsv = new Mat();
        private final Mat rgb = new Mat();
        private final Mat maskY = new Mat();
        private final Mat maskB = new Mat();
        private final Mat maskR1 = new Mat();
        private final Mat maskR2 = new Mat();
        private final Mat maskR = new Mat();
        private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

        // Atomic snapshot so OpMode can read data safely
        private final AtomicReference<DetectionSnapshot> latest = new AtomicReference<>(new DetectionSnapshot(null, null, null, null, null, null));

        public static class DetectionSnapshot {
            public final Rect yellowRect, blueRect, redRect;
            public final Point yellowCenter, blueCenter, redCenter;

            public DetectionSnapshot(Rect yRect, Point yCenter, Rect bRect, Point bCenter, Rect rRect, Point rCenter) {
                this.yellowRect = copyRect(yRect);
                this.yellowCenter = copyPoint(yCenter);
                this.blueRect = copyRect(bRect);
                this.blueCenter = copyPoint(bCenter);
                this.redRect = copyRect(rRect);
                this.redCenter = copyPoint(rCenter);
            }

            private static Rect copyRect(Rect r) {
                if (r == null) return null;
                return new Rect(r.x, r.y, r.width, r.height);
            }
            private static Point copyPoint(Point p) {
                if (p == null) return null;
                return new Point(p.x, p.y);
            }
        }

        public DetectionSnapshot getLatest() {
            return latest.get();
        }

        @Override
        public Mat processFrame(Mat input) {
            // Convert RGBA -> RGB -> HSV (two-step is more portable across OpenCV builds)
            Imgproc.cvtColor(input, rgb, Imgproc.COLOR_RGBA2RGB);
            Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV);

            // Yellow mask
            Core.inRange(hsv, Y_LOWER, Y_UPPER, maskY);
            Imgproc.morphologyEx(maskY, maskY, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(maskY, maskY, Imgproc.MORPH_CLOSE, kernel);

            // Blue mask
            Core.inRange(hsv, B_LOWER, B_UPPER, maskB);
            Imgproc.morphologyEx(maskB, maskB, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(maskB, maskB, Imgproc.MORPH_CLOSE, kernel);

            // Red mask (combine two ranges)
            Core.inRange(hsv, R1_LOWER, R1_UPPER, maskR1);
            Core.inRange(hsv, R2_LOWER, R2_UPPER, maskR2);
            Core.bitwise_or(maskR1, maskR2, maskR);
            Imgproc.morphologyEx(maskR, maskR, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(maskR, maskR, Imgproc.MORPH_CLOSE, kernel);

            // Find largest contour for each mask
            Rect yRect = findLargestRect(maskY);
            Rect bRect = findLargestRect(maskB);
            Rect rRect = findLargestRect(maskR);

            Point yCenter = (yRect != null) ? new Point(yRect.x + yRect.width / 2.0, yRect.y + yRect.height / 2.0) : null;
            Point bCenter = (bRect != null) ? new Point(bRect.x + bRect.width / 2.0, bRect.y + bRect.height / 2.0) : null;
            Point rCenter = (rRect != null) ? new Point(rRect.x + rRect.width / 2.0, rRect.y + rRect.height / 2.0) : null;

            // annotate frame for driver station view (BGR color order)
            if (yRect != null) {
                Imgproc.rectangle(input, yRect.tl(), yRect.br(), new Scalar(0, 255, 255), 2); // yellow
                Imgproc.circle(input, yCenter, 4, new Scalar(0, 255, 255), -1);
                Imgproc.putText(input, "Y " + String.format("%.0f", yRect.area()), new Point(yRect.x, Math.max(10, yRect.y - 6)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, new Scalar(0, 255, 255), 2);
            }
            if (bRect != null) {
                Imgproc.rectangle(input, bRect.tl(), bRect.br(), new Scalar(255, 0, 0), 2); // blue
                Imgproc.circle(input, bCenter, 4, new Scalar(255, 0, 0), -1);
                Imgproc.putText(input, "B " + String.format("%.0f", bRect.area()), new Point(bRect.x, Math.max(10, bRect.y - 6)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, new Scalar(255, 0, 0), 2);
            }
            if (rRect != null) {
                Imgproc.rectangle(input, rRect.tl(), rRect.br(), new Scalar(0, 0, 255), 2); // red
                Imgproc.circle(input, rCenter, 4, new Scalar(0, 0, 255), -1);
                Imgproc.putText(input, "R " + String.format("%.0f", rRect.area()), new Point(rRect.x, Math.max(10, rRect.y - 6)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, new Scalar(0, 0, 255), 2);
            }

            // publish atomic snapshot (defensive copies done in snapshot constructor)
            latest.set(new DetectionSnapshot(yRect, yCenter, bRect, bCenter, rRect, rCenter));

            return input;
        }

        /**
         * Finds largest contour in mask and returns its bounding rect if >= MIN_CONTOUR_AREA, else null.
         * Releases contour Mats before returning.
         */
        private Rect findLargestRect(Mat mask) {
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            double maxArea = 0;
            Rect maxRect = null;

            for (MatOfPoint c : contours) {
                double area = Imgproc.contourArea(c);
                if (area > maxArea) {
                    maxArea = area;
                    maxRect = Imgproc.boundingRect(c);
                }
            }

            // release resources
            hierarchy.release();
            for (MatOfPoint c : contours) {
                c.release();
            }

            if (maxArea >= MIN_CONTOUR_AREA) {
                return maxRect;
            } else {
                return null;
            }
        }

        /**
         * Release native mats when OpMode stops.
         */
        public void releaseResources() {
            try { hsv.release(); } catch (Exception ignored) {}
            try { rgb.release(); } catch (Exception ignored) {}
            try { maskY.release(); } catch (Exception ignored) {}
            try { maskB.release(); } catch (Exception ignored) {}
            try { maskR1.release(); } catch (Exception ignored) {}
            try { maskR2.release(); } catch (Exception ignored) {}
            try { maskR.release(); } catch (Exception ignored) {}
            try { kernel.release(); } catch (Exception ignored) {}
        }
    }
}
