package org.firstinspires.ftc.teamcode.Auto.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class OpenCV extends OpMode {

    OpenCvWebcam webcam1 = null;

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        webcam1.setPipeline(new examplePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam1.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
            }
        });
    }

    @Override
    public void loop() {

    }

    class examplePipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        double leftavgfin;
        double rightavgfin;
        Mat outPut = new Mat();
        Scalar finding = new Scalar(255.0, 0.0, 0.0);
        Scalar found = new Scalar(0.0, 250.0, 0.0);

        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

            Rect leftRect = new Rect(1, 360, 639, 180);
            Rect rightRect = new Rect(640, 360, 639, 180);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, finding, 2);
            Imgproc.rectangle(outPut, rightRect, finding, 2);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];


            if (leftavgfin > 126 && rightavgfin < 126) {
                telemetry.addLine("Center");
                telemetry.addData("leftavgfin", leftavgfin);
                telemetry.addData("rightavgfin", rightavgfin);
                Imgproc.rectangle(outPut, rightRect, found, 2);
            } else if (leftavgfin < 126 && rightavgfin > 126)
            {
                telemetry.addLine("Left");
                telemetry.addData("leftavgfin", leftavgfin);
                telemetry.addData("rightavgfin", rightavgfin);
                Imgproc.rectangle(outPut, leftRect, found, 2);
            } else
            {
                telemetry.addLine("Right");
                telemetry.addData("leftavgfin", leftavgfin);
                telemetry.addData("rightavgfin", rightavgfin);
                Imgproc.rectangle(outPut, leftRect, found, 2);
                Imgproc.rectangle(outPut, rightRect, found, 2);
            }

            return(outPut);
        }
    }
}