package org.firstinspires.ftc.teamcode.bots;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.KeyPoint;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Size;
import org.opencv.features2d.Feature2D;
import org.opencv.features2d.Features2d;
import org.opencv.features2d.SimpleBlobDetector;
import org.opencv.features2d.SimpleBlobDetector_Params;


import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


public class AimBot extends FSMBot {

    OpenCvCamera camera;
    /*
    static SimpleBlobDetector_Params params = new SimpleBlobDetector_Params();

    public void setParams(SimpleBlobDetector_Params params) {
        this.params = params;
        params.set_filterByArea(true);
        params.set_minArea(7.5f);
        params.set_maxArea(1000.0f);

        params.set_filterByInertia(true);
        params.set_minInertiaRatio(0.0f);
        params.set_maxInertiaRatio(0.50f);
    }


     */
    public AimBot(LinearOpMode opMode) {
        super(opMode);
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
       // setParams(new SimpleBlobDetector_Params());
        initCamera();
    }

    //during runOpMode
    public void initCamera(){
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.setPipeline(new JunctionPipeline());
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        opMode.telemetry.setMsTransmissionInterval(50);
    }

    //whileOpModeisactive
    public double detectDistance() {


        return 0;
    }

    public void correctDistance() {

    }




    public static class JunctionPipeline extends OpenCvPipeline {

        public enum Location {
            LEFT,
            RIGHT,
            NO_PIPE
        }

        private Location location;

        Telemetry telemetry;
        Mat mat = new Mat();
        Mat mat_with_points = new Mat();

        Mat edges = new Mat();
        Mat h = new Mat();




        static final Rect CENTER = new Rect(
                new Point(318,1), //bottom left coordinate
                new Point(322, 480) //top right coordinate
        );

        //SimpleBlobDetector detector = SimpleBlobDetector.create(params);

        @Override
        public Mat processFrame(Mat input){

            List<MatOfPoint> contours = new ArrayList<>();
            //convert image from one color space to another
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
            Scalar lowYellow = new Scalar(23,50,70);
            Scalar highYellow = new Scalar(32, 255, 255);

            //threshold the color yellow (only yellow shows in grayscale)
            Core.inRange(mat, lowYellow, highYellow, mat);

            //SOME EXTRA FILTERS
            Imgproc.blur(mat, mat, new Size(6,6));

            //EDGE DETECTION (CANNY)
            //Imgproc.Canny(mat, edges, 100, 100*3 );

            //detect edges first
            Imgproc.Canny(mat, edges, 100, 100 * 2);

            Imgproc.findContours(edges, contours, h, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

            double maxVal = 0;
            int maxValIdx = 0;

            //parse through every contour every frame to find greatest area
            for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++){
                double contourArea = Imgproc.contourArea(contours.get(contourIdx));

                //check for the greatest area
                if (maxVal < contourArea){
                    maxVal = contourArea;
                    maxValIdx = contourIdx;
                }


            }

            //draws the greatest area
            Imgproc.drawContours(mat, contours, maxValIdx, new Scalar(0,255,0), 3);

            //Imgproc.boundingRect(contours);
            //Imgproc.rectangle(mat, boundingRect, new Scalar (255,0,0));

            /* SIMPLEBLOBDETECTION
            //MatOfKeyPoint keypoints = new MatOfKeyPoint();
            //detector.detect(mat, keypoints);
            //Mat mat_with_points = new Mat();
            //Features2d.drawKeypoints(mat, keypoints, mat_with_points, new Scalar(255,0,0), Features2d.DrawMatchesFlags_DRAW_RICH_KEYPOINTS);
             */

            Scalar lockedOn = new Scalar(0,255,0);
            Scalar notLocked = new Scalar(255,0,0);
            Imgproc.rectangle(mat, CENTER, lockedOn);
            return mat;
        }
    }

}

