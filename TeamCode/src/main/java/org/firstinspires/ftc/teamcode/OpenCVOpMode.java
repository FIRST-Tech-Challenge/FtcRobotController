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

@Autonomous(name="Skystone Detector", group="Auto")
public class OpenCVOpMode extends LinearOpMode {
    OpenCvWebcam webcam;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

//        webcam.setPipeline(new SamplePipeline(telemetry));
        webcam.setPipeline(new SamplePipeline());


        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
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
            String color = "";
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
        final Rect ROI = new Rect(new Point(100, 200), new Point(300, 400));

//        public SamplePipeline (Telemetry t) {
//        public SamplePipeline() {
//            System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
//            telemetry = t;
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
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);

            // getColor(input);
            return input;
        }


        /* public double[] getColor(Mat input) {

            double[] color = new double[3];

            Mat coneRegion = mat.submat(ROI);
            double rVal = (double) Core.sumElems(coneRegion).val[0];
            double gVal = (double) Core.sumElems(coneRegion).val[1];
            double bVal = (double) Core.sumElems(coneRegion).val[2];
            rVal /= ROI.area();
            gVal /= ROI.area();
            bVal /= ROI.area();
            color[0] = rVal;
            color[1] = gVal;
            color[2] = bVal;

            telemetry.addData("Color: ", color);
            mat.release();

            return color;
        } */

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
