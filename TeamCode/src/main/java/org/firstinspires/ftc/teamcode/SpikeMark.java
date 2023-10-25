package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

@Autonomous
public class SpikeMark extends OpMode {
    OpenCvCamera webcam1 = null;
    public int spikelocation = 0;
    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam1.setPipeline(new examplePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);

            }

            @Override
            public void onError(int errorCode) {

            }
        });


    }

    @Override
    public void loop() {

    }

    class examplePipeline extends OpenCvPipeline {

        Mat mat = new Mat();
        double percent_color_threshold = 0.04;
        Scalar rectColor1 = new Scalar(255.0, 0.0, 0.0);
        Rect cropped = new Rect(1, 260, 639, 479);

        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
            Scalar lowHSV = new Scalar(150, 50, 70);
            Scalar highHSV = new Scalar(179, 255, 255);

            Core.inRange(mat, lowHSV, highHSV, mat);

            Rect leftRect = new Rect(1, 260, 159, 219);
            Rect midRect = new Rect(160, 260, 319, 219);
            Rect rightRect = new Rect(480, 260, 159, 219);


            Mat left = mat.submat(leftRect);
            Mat mid = mat.submat(midRect);
            Mat right = mat.submat(rightRect);

            double leftval = Core.sumElems(left).val[0] / leftRect.area() / 255 - 0.05;
            double midval = Core.sumElems(mid).val[0] / midRect.area() / 255 - 0.02;
            double rightval = Core.sumElems(right).val[0] / rightRect.area() / 255 - 0.05;

            Imgproc.rectangle(mat, leftRect, rectColor1, 2);
            Imgproc.rectangle(mat, rightRect, rectColor1, 2);
            Imgproc.rectangle(mat, midRect, rectColor1, 2);

            left.release();
            mid.release();
            right.release();

            telemetry.addData("left raw: ", leftval * leftRect.area() * 255);
            telemetry.addData("mid raw: ", midval * midRect.area() * 255);
            telemetry.addData("right raw: ", rightval * rightRect.area() * 255);

            telemetry.addData("left %", Math.round(leftval * 100));
            telemetry.addData("mid %", Math.round(midval * 100));
            telemetry.addData("right %", Math.round(rightval * 100));

            if (leftval > rightval && leftval > midval) spikelocation = 1;
            else if (midval > leftval && midval > rightval) spikelocation = 2;
            else if (rightval > leftval && rightval > midval) spikelocation = 3;
            else spikelocation = 0;

            /*
            boolean posleft  = leftval > percent_color_threshold;
            boolean posmid  = midval > percent_color_threshold;
            boolean posright  = rightval > percent_color_threshold;

            if (posleft) telemetry.addData("left position", "yes");
            else if (posmid) telemetry.addData("mid position", "yes");
            else if (posright) telemetry.addData("right position", "Yes");
            else telemetry.addData("not found", "bruh");
            */
            return(mat);
        }
    }
}

/*
 Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat midCrop;
        Mat rightCrop;
        double leftavgfin;
        double midavgfin;
        double rightavgfin;
        Mat output = new Mat();
        Scalar rectColor1 = new Scalar(255.0, 0.0, 0.0);
        Scalar rectColor2 = new Scalar(0.0, 255.0, 0.0);
        Scalar rectColor3 = new Scalar(0.0, 0.0, 255.0);

        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);



            Rect leftRect = new Rect(1, 1, 159, 479);
            Rect midRect = new Rect(160, 1, 319, 479);
            Rect rightRect = new Rect(480, 1, 159, 479);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColor1, 2);
            Imgproc.rectangle(output, midRect, rectColor3, 2);
            Imgproc.rectangle(output, rightRect, rectColor2, 2);

            leftCrop = YCbCr.submat(leftRect);
            midCrop = YCbCr.submat(midRect);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);
            Core.extractChannel(midCrop, midCrop, 2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);
            Scalar midavg = Core.mean(midCrop);


            leftavgfin = leftavg.val[0];
            midavgfin = midavg.val[0];
            rightavgfin = rightavg.val[0];

            telemetry.addLine("leftavgfin: " + leftavgfin);
            telemetry.addLine("rightavgfin: " + rightavgfin);
            telemetry.addLine("midavgfin: " + midavgfin);
            if (leftavgfin < rightavgfin && leftavgfin < midavgfin) telemetry.addLine("Left");
            else if (midavgfin < rightavgfin && midavgfin < leftavgfin) telemetry.addLine("Middle");
            else if (rightavgfin < leftavgfin && rightavgfin < midavgfin) telemetry.addLine("Right");


            return(output);


        }
    }
 */