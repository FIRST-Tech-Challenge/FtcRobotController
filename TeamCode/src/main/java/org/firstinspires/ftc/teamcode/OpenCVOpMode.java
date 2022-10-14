package org.firstinspires.ftc.teamcode;

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
import org.openftc.easyopencv.*;

import java.util.HashMap;

@Autonomous(name="Camera Color Sensor", group="Auto")
public class OpenCVOpMode extends LinearOpMode {
    OpenCvWebcam webcam;
    CameraColorPipeline pipeline;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new CameraColorPipeline();
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addLine("Error, onError() function activated");
                telemetry.update();
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();

            if(gamepad1.a)
            {
                webcam.stopStreaming();
            }

            sleep(100);
        }
    }

    class CameraColorPipeline extends OpenCvPipeline
    {
        boolean viewportPaused;
        int frameCount = 0;
        final Rect ROI = new Rect(new Point(155, 115), new Point(165, 125));

        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4.0,
                            input.rows()/4.0),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);

            String colorName = getColor(input);
            frameCount++;

            if (frameCount == 20) {
                if (colorName.equals("orange")) {
                    Auton auton = new Auton(1);
                } else if (colorName.equals("teal")) {
                    Auton auton = new Auton(2);
                } else {
                    Auton auton = new Auton(3);
                }
            }

            return input;
        }

        public String getColor(Mat input) {
            int[] camValues = new int[3];

            Mat coneRegion = input.submat(ROI);
            camValues[0] = (int) Core.sumElems(coneRegion).val[0] / (int) ROI.area();
            camValues[1] = (int) Core.sumElems(coneRegion).val[1] / (int) ROI.area();
            camValues[2] = (int) Core.sumElems(coneRegion).val[2] / (int) ROI.area();
            String colorString = camValues[0] + ", " + camValues[1] + ", " + camValues[2];
            telemetry.addData("Color in RGB: ", colorString);

            String colorName = MSE(camValues);
            telemetry.addData("Closest Color: ", colorName);
            coneRegion.release();

            return colorName;
        }

        public String MSE(int[] camValues) {
            int[][] coneColorValues = {{255, 165, 0}, {135, 206, 235}, {255, 0, 255}};
            // in order orange, teal, pink ^^^^^^^

            int[] diffs = new int[3];

            for (int i = 0; i < 3; i++) {
                int rDiff = (int) Math.pow(coneColorValues[i][0] - camValues[0], 2);
                int gDiff = (int) Math.pow(coneColorValues[i][1] - camValues[1], 2);
                int bDiff = (int) Math.pow(coneColorValues[i][2] - camValues[2], 2);
                diffs[i] = rDiff + gDiff + bDiff;
            }

            String diffsString = diffs[0] + ", " + diffs[1] + ", " + diffs[2];
            telemetry.addData("diffs: ", diffsString);

            if (diffs[1] < diffs[2] && diffs[1] < diffs[0]) { return "teal"; }
            else if (diffs[2] < diffs[0] && diffs[2]< diffs[1]) { return "pink"; }
            else { return "orange"; }
        }

        @Override
        public void onViewportTapped()
        {
            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}
