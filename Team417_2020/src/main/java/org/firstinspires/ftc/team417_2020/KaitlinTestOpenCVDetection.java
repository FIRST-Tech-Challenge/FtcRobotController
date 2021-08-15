package org.firstinspires.ftc.team417_2020;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Test Detection")
public class KaitlinTestOpenCVDetection extends LinearOpMode {

    OpenCvCamera webcam;
    TestPipeLine detector = new TestPipeLine();
    int numRings;

    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Side Webcam"), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
        });
        webcam.setPipeline(detector);

        /*
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(new SamplePipeline() );
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
        */

        telemetry.addLine("Waiting for start");

        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            //telemetry.addData("contours:", TestPipeLine.contours.size());
            // X, Y, widthxheight
            // todo crop image to be where rings rest on field
            /*
            * 4 rings : {229, 298, 178x127} r = 1.4
            * 1 ring  : {230, 371, 176x54}  r = 3.3
             */
            telemetry.addData("max rectangle", detector.maxRect.toString());
            if (detector.maxRect.width / detector.maxRect.height > 2.5) {
                numRings = 1;
            } else {
                numRings = 4;
            }
            telemetry.addData("Number of rings", numRings);
            telemetry.update();
            idle();
        }

        // close camera
        webcam.stopStreaming();
        webcam.closeCameraDevice();


    }

    static class TestPipeLine extends OpenCvPipeline {
        // store orange to Rect if area greater than ___
        // if Rect null then 0



        private Mat displayMat = new Mat();
        private Mat yuv = new Mat();
        private Mat u = new Mat();
        private Mat blurred = new Mat();

        private Mat hierarchy = new Mat();
        Rect rectCrop = new Rect(230, 300, 180, 130);

        Rect rect = new Rect();
        Rect maxRect = new Rect(0,0,1,1);

        @Override
        public Mat processFrame(Mat input) {

            // copy input frame to different Mat as to not alter input
            input.copyTo(displayMat);
            //displayMat = displayMat.submat(rectCrop);
            // blur image
            Imgproc.GaussianBlur(input, blurred, new Size(15,15), 0);
            // convert color space to YUV
            Imgproc.cvtColor(blurred, yuv,Imgproc.COLOR_BGR2YUV);
            // split blurred YUV image into YUV channels

            Core.extractChannel(yuv,u,2);
            //u = u.submat(rectCrop);
            Imgproc.threshold(u, u, 100,255,Imgproc.THRESH_BINARY_INV);
            List<MatOfPoint> contours = new ArrayList<>();
            // find contours
            Imgproc.findContours(u, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            /*for (int i = 0; i < contours.size(); i++) {
                Imgproc.drawContours(displayMat, contours, i, new Scalar(0,0,0), 2);
            }

            */
            // find largest contour and assign to variable maxRect

            double maxArea = 0.0;
            for (MatOfPoint contour: contours){
                rect = Imgproc.boundingRect(contour);
                Imgproc.rectangle(displayMat,
                        new Point(rect.x, rect.y),
                        new Point(rect.x + rect.width, rect.y + rect.height),
                        new Scalar(0,0,0), 4);
                double area = Imgproc.contourArea(contour);
                if (area > maxArea){
                    maxArea = area;
                    maxRect = rect;
                }
            }
            double x = maxRect.x;
            double y = maxRect.y;
            // draw rectangle around largest contour
            Imgproc.rectangle(displayMat,
                    new Point(x, y),
                    new Point(x + maxRect.width, y + maxRect.height),
                    new Scalar(0,0,0), 4);
            Imgproc.putText(displayMat, "" + maxRect.toString(), new Point(x,y), 0, 1, new Scalar(0,0,0));


            return displayMat;
            //return u;
        }




    }
}
