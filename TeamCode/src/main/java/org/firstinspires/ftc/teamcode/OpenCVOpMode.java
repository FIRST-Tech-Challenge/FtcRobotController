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

@Autonomous(name="Skystone Detector", group="Auto")
public class OpenCVOpMode extends LinearOpMode {
    OpenCvWebcam webcam;
    SamplePipeline pipeline;


    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new SamplePipeline();
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
                telemetry.addLine("Error lmao");
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

    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;
//        Telemetry telemetry;
        Mat mat = new Mat();
        // not exact numbers yet
        final int[][] colorVals = {{255, 165, 0}, {135, 206, 235}, {255, 0, 255}}; // orange, blue, pink
        final Rect ROI = new Rect(new Point(0, 0), new Point(100, 100));

//        public SamplePipeline() {
//            System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
//        }

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4.0,
                            input.rows()/4.0),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);

            int[] cameraColors = getColor(input);
            colorDet(colorVals, cameraColors);
            return input;
        }


        public int[] getColor(Mat input) {
            int[] res = new int[3];

            Mat coneRegion = input.submat(ROI);
            int rVal = (int)Core.sumElems(coneRegion).val[0];
            int gVal = (int)Core.sumElems(coneRegion).val[1];
            int bVal = (int)Core.sumElems(coneRegion).val[2];
            rVal /= ROI.area();
            gVal /= ROI.area();
            bVal /= ROI.area();

            String colorString = rVal + ", " + gVal + ", " + bVal;

            telemetry.addData("Color: ", colorString);
            coneRegion.release();
            res[0] = rVal;
            res[1] = gVal;
            res[2] = bVal;
            return res;
        }

        public int MSE(int index, int[][] colorVals, int[] cameraColors) {
            int diff_r = (cameraColors[0] - colorVals[index][0])^2;
            int diff_g = (cameraColors[1] - colorVals[index][1])^2;
            int diff_b = (cameraColors[2] - colorVals[index][2])^2;

            return diff_r + diff_g + diff_b;
        }


        public String colorDet(int[][] colorVals, int[] cameraColors) {
            String[] dets = {"orange", "blue", "pink"};
            String res = "";
            // MSE: (1/n) * Σ(colorVals - cameraColors)^2

            double orange_MSE = MSE(0, colorVals, cameraColors);
            double blue_MSE = MSE(1, colorVals, cameraColors);
            double pink_MSE = MSE(2, colorVals, cameraColors);

            double min = Math.min(Math.min(orange_MSE, blue_MSE), pink_MSE);

            if ((int) min == (int) orange_MSE) res = "orange";
            if ((int) min == (int) blue_MSE) res = "blue";
            if ((int) min == (int) pink_MSE) res = "pink";



            telemetry.addData("orange_MSE: ", orange_MSE);
            telemetry.addData("blue_MSE: ", blue_MSE);
            telemetry.addData("pink_MSE: ", pink_MSE);
            telemetry.addData("closest color: ", res);
            return res;
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
