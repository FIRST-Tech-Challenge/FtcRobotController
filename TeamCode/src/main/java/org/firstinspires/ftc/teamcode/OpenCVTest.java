package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;
import org.opencv.core.*;

@Autonomous
public class OpenCVTest extends OpMode {

    class DetectColor extends OpenCvPipeline {
        Mat YCrCb = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        double leftAverageFinal;
        double rightAverageFinal;
        Mat output = new Mat();
        final Scalar RED = new Scalar(255, 0, 0);
        final Scalar GREEN = new Scalar(0, 255, 0);

        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("DetectColor Pipeline Running");

            Rect leftRect = new Rect(1, 1, 319, 359);
            Rect rightRect = new Rect(320, 1, 319, 359);

            Imgproc.rectangle(output, leftRect, RED, 2);
            Imgproc.rectangle(output, rightRect, RED, 2);

            leftCrop = YCrCb.submat(leftRect);
            rightCrop = YCrCb.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);

            Scalar leftAverage = Core.mean(leftCrop);
            Scalar rightAverage = Core.mean(rightCrop);

            leftAverageFinal = leftAverage.val[0];
            rightAverageFinal = rightAverage.val[0];

            if (leftAverageFinal > rightAverageFinal) {
                Imgproc.rectangle(output, leftRect, GREEN, 2);
                telemetry.addLine("Red object detected in left.");
            }
            else if (leftAverageFinal < rightAverageFinal) {
                Imgproc.rectangle(output, rightRect, GREEN, 2);
                telemetry.addLine("Red object detected in right.");
            }
            else {
                telemetry.addLine("Red amount in both is equal.");
            }

            telemetry.addData("Left Average Red Value: ", leftAverageFinal);
            telemetry.addData("Right Average Red Value: ", rightAverageFinal);

            return output;
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
                camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                System.out.println("it no work");
            }
        });
    }

    @Override
    public void loop() {
        telemetry.update();
    }

}