package org.firstinspires.ftc.team6220_CENTERSTAGE;

import org.opencv.core.Core;
import org.opencv.core.Mat;

import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.openftc.easyopencv.OpenCvPipeline;



public abstract class colorDetectTestOpenCV extends OpenCvPipeline {

    Mat test = new Mat();

    public Mat processFrame(Mat input) {


        // Initialize the VideoCapture object for the default webcam (usually camera 0)
        VideoCapture capture = new VideoCapture(0);

        // Check if the camera opened successfully
        if (!capture.isOpened()) {
            System.out.println("Error: Camera not found!");
            System.exit(404);
        }

        Imgproc.cvtColor(input, test, Imgproc.COLOR_RGB2HSV);




        return test;

    }

}
