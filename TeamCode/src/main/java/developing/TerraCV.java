package developing;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TerraCV extends OpenCvPipeline
{

    public double ORANGE_MIN = 80;
    public double ORANGE_MAX = 110;

    // Cases
    public enum RingNum {ZERO, ONE, FOUR}


    // Thresholds
    public static int HEIGHT_MIN = 10;
    public static int WIDTH_MIN = 15;
    public static int HEIGHT_MAX = 60;
    public static int WIDTH_MAX = 60;
    public static double ONE_MIN = 2.3;
    public static double ONE_MAX = 2.8;
    public static double FOUR_MIN = 0.5;
    public static double FOUR_AREA = 1000;

    public double[] result = new double[3];



    public Mat yCrCb = new Mat();
    public Mat cb = new Mat();
    public Mat processed = new Mat();

    public List<MatOfPoint> contours = new ArrayList<>();

    public RingNum ringNum;

    public MatOfPoint2f areaPoints;
    public RotatedRect boundingRect;

    @Override
    public Mat processFrame(Mat input)
    {
        contours.clear();

        Imgproc.cvtColor(input, yCrCb, Imgproc.COLOR_RGB2YCrCb);

        Core.extractChannel(yCrCb, cb, 2);

        Core.inRange(cb, new Scalar(ORANGE_MIN), new Scalar(ORANGE_MAX), processed);

        Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());


        Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        int i = 0;
        for (MatOfPoint contour : contours) {
            areaPoints = new MatOfPoint2f(contour.toArray());
            boundingRect = Imgproc.minAreaRect(areaPoints);

            if (HEIGHT_MIN < boundingRect.size.height && boundingRect.size.height < HEIGHT_MAX && WIDTH_MIN < boundingRect.size.width && boundingRect.size.width < WIDTH_MAX) {

                Imgproc.rectangle(input, boundingRect.boundingRect(), new Scalar(0, 255, 0), 4);
                i++;

                double width = boundingRect.size.width;
                double height = boundingRect.size.height;
                double wh_ratio = width/height;

                result = new double[] {width, height, wh_ratio};

                if (FOUR_MIN <= wh_ratio && wh_ratio <= ONE_MIN && boundingRect.size.area() >= FOUR_AREA) {
                    ringNum = RingNum.FOUR;
                } else if (ONE_MIN <= wh_ratio && wh_ratio <= ONE_MAX) {
                    ringNum = RingNum.ONE;
                }else{
                    ringNum = RingNum.ZERO;
                }
            }
        }

        if(i == 0){
            ringNum = RingNum.ZERO;
        }
        return processed;
    }

    @Override
    public void onViewportTapped()
    {


    }
}
