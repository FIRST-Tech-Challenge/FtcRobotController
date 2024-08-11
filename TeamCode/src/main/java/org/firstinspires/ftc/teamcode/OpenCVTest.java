package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.*;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

@Autonomous
public class OpenCVTest extends OpMode {

    static class Gray extends OpenCvPipeline {
        // Notice this is declared as an instance variable (and re-used), not a local variable
        Mat grey = new Mat();

        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);
            return grey;
        }
    }

    public void init() {
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Camera");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcam, cameraMonitorViewId);
        camera.setPipeline(new Gray());

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