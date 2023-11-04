package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

import java.lang.annotation.Target;

public class SpikeCam {
    OpenCvCamera webcam1 = null;
    public enum location{
        LEFT,
        MIDDLE,
        RIGHT,
        NOT_FOUND
    }

    public enum TargetColor{
        BLUE,RED
    }

    private TargetColor myColor;
    location spikeLocation = location.NOT_FOUND;
    private LinearOpMode myOpMode;

    public void initialize(LinearOpMode currentOp, TargetColor targetColor) {
        myOpMode = currentOp;
        HardwareMap hardwareMap = myOpMode.hardwareMap;
        myColor = targetColor;

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam1.setPipeline(new SpikeCam.spikePipeline());

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

    public location getSpikeLocation() {
        return spikeLocation;
    }

    class spikePipeline extends OpenCvPipeline {

        Mat mat = new Mat();
        Scalar rectColor1 = new Scalar(255.0, 0.0, 0.0);
        Rect cropped = new Rect(1, 260, 639, 479);

        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
            Scalar lowHSV = new Scalar(150, 50, 70);
            Scalar highHSV = new Scalar(179, 255, 255);

            if(myColor == TargetColor.RED) {
                lowHSV = new Scalar(150, 50, 70);
                highHSV = new Scalar(179, 255, 255);
            } else {
                lowHSV = new Scalar(105, 50, 70);
                highHSV = new Scalar(135, 255, 255);
            }
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

        /*    myOpMode.telemetry.addData("left raw: ", leftval * leftRect.area() * 255);
            myOpMode.telemetry.addData("mid raw: ", midval * midRect.area() * 255);
            myOpMode.telemetry.addData("right raw: ", rightval * rightRect.area() * 255);

            myOpMode.telemetry.addData("left %", Math.round(leftval * 100));
            myOpMode.telemetry.addData("mid %", Math.round(midval * 100));
            myOpMode.telemetry.addData("right %", Math.round(rightval * 100));
*/
            if (leftval > rightval && leftval > midval) spikeLocation = location.LEFT;
            else if (midval > leftval && midval > rightval) spikeLocation = location.MIDDLE;
            else if (rightval > leftval && rightval > midval) spikeLocation = location.RIGHT;
            else spikeLocation = location.NOT_FOUND;


            return(mat);
        }
    }

}

         /*
            boolean posleft  = leftval > percent_color_threshold;
            boolean posmid  = midval > percent_color_threshold;
            boolean posright  = rightval > percent_color_threshold;

            if (posleft) telemetry.addData("left position", "yes");
            else if (posmid) telemetry.addData("mid position", "yes");
            else if (posright) telemetry.addData("right position", "Yes");
            else telemetry.addData("not found", "bruh");
            */