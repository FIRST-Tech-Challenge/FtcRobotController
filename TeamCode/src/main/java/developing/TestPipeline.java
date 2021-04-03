package developing;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class TestPipeline extends OpenCvPipeline
{

    public Mat yCbCrChan2Mat = new Mat();
    public Mat thresholdMat = new Mat();
    public Mat contoursOnFrameMat = new Mat();
    public ArrayList<MatOfPoint> contoursList = new ArrayList<>();
    public int numContoursFound;


    @Override
    public Mat processFrame(Mat input)
    {
        contoursList.clear();

        Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);
        Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);
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
