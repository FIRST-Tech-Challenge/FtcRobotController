package org.firstinspires.ftc.team6220_CENTERSTAGE;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;


import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;



public abstract class colorDetectTestOpenCV extends OpenCvPipeline {

    OpenCvCamera robotCamera;

    Mat test = new Mat();

    public Mat processFrame(Mat input) {


        // Initialize the VideoCapture object for the default webcam (usually camera 0)
        robotCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"));


        // Check if the camera opened successfully
        if (!capture.isOpened()) {
            System.out.println("Error: Camera not found!");
            System.exit(404);
        }

        Imgproc.cvtColor(input, test, Imgproc.COLOR_RGB2HSV);




        return test;

    }

}
