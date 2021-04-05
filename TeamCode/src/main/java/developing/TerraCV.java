package developing;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TerraCV extends OpenCvPipeline
{
    //Cases
    public enum RingNum {ZERO, ONE, FOUR}
    //frame after processing
    public Mat processed = new Mat();
    //RingNum detected
    public RingNum ringNum;

    //Hsv frame
    public Mat hsv = new Mat();
    //Position of rings on screen use telemetry to change
    // Note: (x is actually y pos and starts at 0 from bottom, ypos starts at 0 from left and is x
    public int xPos = 75;
    public int yPos = 500;

    //average color and hue value
    public double[] avgColor = new double[2];
    public double avgH;



    @Override
    public Mat processFrame(Mat input)
    {
        input.convertTo(input, -1, 2, 100); //Artificially increase brightness
        Rect rectCrop = new Rect(xPos, yPos, 150,150); //define rect to crop image based on xpos and ypos
        processed = new Mat(input, rectCrop); //crop
        Imgproc.cvtColor(processed, hsv, Imgproc.COLOR_RGB2HSV); //convert to hsv color space
        avgColor = Core.mean(hsv).val; //find the mean value of the colors
        avgH = avgColor[0]; // find the mean hue value

        if(avgH > 90){ //for zero hue is usually around 100
            ringNum = RingNum.ZERO;
        }else if(avgH > 50){ //for one hue is usually around 70
            ringNum = RingNum.ONE;
        }else if(avgH > 10){ // for four hue is usually around 30
            ringNum = RingNum.FOUR;
        }
        //Uncomment this line if you want to view fullscreen
        //Imgproc.resize(processed, processed, input.size());

        return processed;
    }
}
