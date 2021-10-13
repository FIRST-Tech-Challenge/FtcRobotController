package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp
public class ThresholdingTest extends LinearOpMode {
    OpenCvCamera webcam;
    Threshold thresh;

    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,  "Webcam 1"), cameraMonitorViewId);

        thresh = new Threshold();
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.setPipeline(thresh);
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){

        }
    }

    class Threshold extends OpenCvPipeline{

        int stage = 0;
        Mat ycrcb = new Mat();
        Mat threshold = new Mat();

        @Override
        public void onViewportTapped() {
            stage ++;

            if(stage > 2){
                stage = 0;
            }
        }

        @Override
        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(ycrcb, ycrcb,0);
            Imgproc.threshold(ycrcb, threshold, 255 * gamepad1.right_trigger, 100, Imgproc.THRESH_TOZERO);

            switch (stage){
                case 0:
                    return ycrcb;
                case 1:
                    return threshold;
                case 2:
                    return input;
                default:
                    return  input;
            }
        }
    }
}
