package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

@Autonomous(name = "Right Auton")
public class rightAuton extends LinearOpMode {
    OpenCvWebcam webcam;
    CameraColorPipeline pipeline;
    int currentColor = 0;
    String colorName;

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
                telemetry.addLine("Camera Opened");
                telemetry.update();
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

        waitForStart();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addData("Color", currentColor);
        telemetry.update();
        Auton auto = new Auton(false);
        auto.runAuton(currentColor, drive);



    }

    class CameraColorPipeline extends OpenCvPipeline
    {
        boolean viewportPaused;
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
            if (colorName.equals("red")) {
                currentColor=1;
            } else if (colorName.equals("green")) {
                currentColor=2;
            } else { // blue
                currentColor=3;
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
            int[][] coneColorValues = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}};
            // red, green, blue

            int[] diffs = new int[3];

            for (int i = 0; i < 3; i++) {
                int rDiff = (int) Math.pow(coneColorValues[i][0] - camValues[0], 2);
                int gDiff = (int) Math.pow(coneColorValues[i][1] - camValues[1], 2);
                int bDiff = (int) Math.pow(coneColorValues[i][2] - camValues[2], 2);
                diffs[i] = rDiff + gDiff + bDiff;
            }

            String diffsString = diffs[0] + ", " + diffs[1] + ", " + diffs[2];
            telemetry.addData("diffs: ", diffsString);

            if (diffs[1] < diffs[2] && diffs[1] < diffs[0]) { return "green"; }
            else if (diffs[2] < diffs[0] && diffs[2]< diffs[1]) { return "blue"; }
            else { return "red"; }
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
