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

    public Mat hsvMat = new Mat();
    public Mat thresholdMat = new Mat();
    public Mat contoursOnFrameMat = new Mat();
    public ArrayList<MatOfPoint> contoursList = new ArrayList<>();
    public int numContoursFound;

    public Scalar ringColor =  new Scalar(30,86,82);
    Scalar lower =  new Scalar(20,64,64);
    Scalar upper =  new Scalar(40,100,100);


    @Override
    public Mat processFrame(Mat input)
    {
        contoursList.clear();


        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
//
//        Core.extractChannel(hsvMat, hsvMat, 0);


        Core.inRange(hsvMat, lower, upper, thresholdMat);
//        Imgproc.threshold(hsvMat, thresholdMat, 102, 100, Imgproc.THRESH_BINARY_INV);
        Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        numContoursFound = contoursList.size();
        input.copyTo(contoursOnFrameMat);
        Imgproc.drawContours(contoursOnFrameMat, contoursList, -1, new Scalar(0, 0, 255), 3, 8);


        return contoursOnFrameMat;
    }

    @Override
    public void onViewportTapped()
    {


    }
}
