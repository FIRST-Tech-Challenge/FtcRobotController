package org.firstinspires.ftc.teamcode.JackBurr.Camera.OpenCV;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class EdgeDetectionPipeline extends OpenCvPipeline {
    Mat gray = new Mat(); //create grayscale mat
    Mat edges = new Mat(); // create edges mat

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);  //takes the input Mat, converts it to grayscale, and outputs it to gray Mat
        Imgproc.Canny(gray, edges, 50, 100); // uses canny edge detection on the gray Mat and outputs it to edges Mat
        return edges;
    }
}