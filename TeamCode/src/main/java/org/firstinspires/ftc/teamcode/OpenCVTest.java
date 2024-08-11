package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;
import org.opencv.core.*;

@Autonomous
public class OpenCVTest extends OpMode {

    static class DetectColor extends OpenCvPipeline {

        // point constants for the top left half and the bottom right half of the screen
        static final Point topLeft = new Point(320.0/4.0, 240.0/4.0);
        static final Point bottomRight = new Point(320.0*0.75, 240.0*0.75);
        // color constants
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        Mat RGB = new Mat();
        Mat G = new Mat();
        Mat region1_G;
        int avgColor;

        @Override
        public void init(Mat firstFrame) {
            Core.extractChannel(RGB, G, 0);
            region1_G = G.submat(new Rect(topLeft, bottomRight));
        }

        @Override
        public Mat processFrame(Mat input) {
            Core.extractChannel(RGB, G, 0);
            avgColor = (int) Core.mean(region1_G).val[0];

            if (avgColor < 100) {
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        topLeft, // First point which defines the rectangle
                        bottomRight, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        2); // Negative thickness means solid fill
            }
            else {
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        topLeft, // First point which defines the rectangle
                        bottomRight, // Second point which defines the rectangle
                        RED, // The color the rectangle is drawn in
                        2); // Thickness of the rectangle lines
            }

            return input;
        }

    }

    public void init() {
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Camera");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId);
        camera.setPipeline(new DetectColor());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                System.out.println("it no work");
            }
        });
    }

    @Override
    public void loop() {

    }

}