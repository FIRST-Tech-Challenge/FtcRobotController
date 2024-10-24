package org.firstinspires.ftc.teamcode.Vision.imageCapture;

import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CaptureImagePipeline extends OpenCvPipeline {

    Mat outputMat = new Mat();
    Mat save = new Mat();
    int imgIndex = 0;

    String path = "/storage/self/primary/Pictures/";

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(outputMat);
        return outputMat;
    }

    public void saveMatToDisk(String name) {
        outputMat.copyTo(save);
        Imgproc.cvtColor(save, save, Imgproc.COLOR_BGR2RGB);
        Imgcodecs.imwrite(path + name + imgIndex, save);
        imgIndex++;
    }

    public void saveMatToDisk() {
        outputMat.copyTo(save);
        Imgproc.cvtColor(save, save, Imgproc.COLOR_BGR2RGB);
        Imgcodecs.imwrite(path + "capture" + imgIndex + ".png", save);
        imgIndex++;
    }
}