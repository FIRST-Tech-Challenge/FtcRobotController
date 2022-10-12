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
        Mat mat = new Mat();
        final Rect ROI = new Rect(new Point(0, 0), new Point(320, 240));

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

            getColor(input);
            return input;
        }


        public void getColor(Mat input) {
            int[] res = new int[3];
            int[] camValues = new int[3];

            Mat coneRegion = input.submat(ROI);
            camValues[0] = (int)Core.sumElems(coneRegion).val[0] / (int)ROI.area();
            camValues[1] = (int)Core.sumElems(coneRegion).val[1] / (int)ROI.area();
            camValues[2] = (int)Core.sumElems(coneRegion).val[2] / (int)ROI.area();
            String colorString = camValues[0] + ", " + camValues[1] + ", " + camValues[2];
            telemetry.addData("Color: ", colorString);

            String colorName = MSE(camValues);
            telemetry.addData("Color: ", colorName);

            coneRegion.release();
        }

        public String MSE(int[] colors) {
            int[][] coneColorValues = {{255, 165, 0}, {135, 206, 235}, {255, 0, 255}};
            // in order orange, teal, pink ^^^^^^^

            int[] diffs = new int[3];

            for(int i = 0; i < 3; i++) {
                int rDif = (coneColorValues[i][0] - colors[0]) * (coneColorValues[i][0] - colors[0]);
                int gDif = (coneColorValues[i][1] - colors[1]) * (coneColorValues[i][1] - colors[1]);
                int bDif = (coneColorValues[i][2] - colors[2]) * (coneColorValues[i][2] - colors[2]);
                diffs[i] = rDif + gDif + bDif;
            }

            String diffsString = diffs[0] + ", " + diffs[1] + ", " + diffs[2];
            telemetry.addData("diffs: ", diffsString);

            if(diffs[1] < diffs[2] && diffs[1] < diffs[0]) { return "teal"; }
            else if(diffs[2] < diffs[0]) { return "pink"; }
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
