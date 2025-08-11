package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@TeleOp(name="Color & Position Detection", group="Vision")
public class ColorPositionDetectionOpMode extends OpMode {

    OpenCvCamera camera;
    DetectionPipeline pipeline;

    @Override
    public void init() {
        // Camera setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createInternalCamera(
                OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId
        );

        pipeline = new DetectionPipeline();
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
            }
        });
    }

    @Override
    public void loop() {
        telemetry.addData("Detected Color", pipeline.detectedColor);
        telemetry.addData("Object X", pipeline.objectCenter.x);
        telemetry.addData("Object Y", pipeline.objectCenter.y);
        telemetry.update();
    }

    // ---------------- PIPELINE CLASS ----------------
    class DetectionPipeline extends OpenCvPipeline {
        String detectedColor = "None";
        Point objectCenter = new Point(0, 0);

        Mat hsv = new Mat();
        Mat mask = new Mat();

        // HSV Ranges for colors (tune for your lighting)
        Scalar lowerRed1 = new Scalar(0, 100, 100);
        Scalar upperRed1 = new Scalar(10, 255, 255);
        Scalar lowerRed2 = new Scalar(160, 100, 100);
        Scalar upperRed2 = new Scalar(179, 255, 255);

        Scalar lowerBlue = new Scalar(100, 150, 0);
        Scalar upperBlue = new Scalar(140, 255, 255);

        Scalar lowerYellow = new Scalar(20, 100, 100);
        Scalar upperYellow = new Scalar(30, 255, 255);

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Detect RED
            Mat mask1 = new Mat();
            Mat mask2 = new Mat();
            Core.inRange(hsv, lowerRed1, upperRed1, mask1);
            Core.inRange(hsv, lowerRed2, upperRed2, mask2);
            Core.add(mask1, mask2, mask);

            if (Core.countNonZero(mask) > 500) {
                detectedColor = "Red";
                objectCenter = findCenter(mask);
            }
            // Detect BLUE
            if (detectColor(hsv, lowerBlue, upperBlue, "Blue")) {
                // done in helper
            }
            // Detect YELLOW
            if (detectColor(hsv, lowerYellow, upperYellow, "Yellow")) {
                // done in helper
            } else {
                detectedColor = "None";
                objectCenter = new Point(0, 0);
            }

            return input;
        }

        private boolean detectColor(Mat hsv, Scalar lower, Scalar upper, String colorName) {
            Core.inRange(hsv, lower, upper, mask);
            if (Core.countNonZero(mask) > 500) {
                detectedColor = colorName;
                objectCenter = findCenter(mask);
                return true;
            }
            return false;
        }

        private Point findCenter(Mat mask) {
            Rect rect = Imgproc.boundingRect(mask);
            return new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
        }
    }
}
