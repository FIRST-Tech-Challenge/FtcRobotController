package developing;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class TerraCV extends OpenCvPipeline
{

    public static double ORANGE_MIN = 80;
    public static double ORANGE_MAX = 110;

    public Mat hsvMat = new Mat();
    public Mat thresholdMat = new Mat();
    public Mat contoursOnFrameMat = new Mat();
    public ArrayList<MatOfPoint> contoursList = new ArrayList<>();
    public int numContoursFound;

    private Mat yCrCb = new Mat();
    private Mat cb = new Mat();
    private Mat processed = new Mat();
//
//    public Scalar ringColor =  new Scalar(30,86,82);
//    Scalar lower =  new Scalar(20,100,100);
//    Scalar upper =  new Scalar(40,255,255);


    @Override
    public Mat processFrame(Mat input)
    {
        contoursList.clear();

        Imgproc.cvtColor(input, yCrCb, Imgproc.COLOR_RGB2YCrCb);

        Core.extractChannel(yCrCb, cb, 2);


        Core.inRange(cb, new Scalar(ORANGE_MIN), new Scalar(ORANGE_MIN), processed);

        Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());

//
//
//        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
////
////        Core.extractChannel(hsvMat, hsvMat, 0);
//
//
//        Core.inRange(hsvMat, lower, upper, thresholdMat);
////        Imgproc.threshold(hsvMat, thresholdMat, 102, 100, Imgproc.THRESH_BINARY_INV);
//        Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
//        numContoursFound = contoursList.size();
//        input.copyTo(contoursOnFrameMat);
//        Imgproc.drawContours(contoursOnFrameMat, contoursList, -1, new Scalar(0, 0, 255), 3, 8);


        return processed;
    }

    @Override
    public void onViewportTapped()
    {


    }
}
