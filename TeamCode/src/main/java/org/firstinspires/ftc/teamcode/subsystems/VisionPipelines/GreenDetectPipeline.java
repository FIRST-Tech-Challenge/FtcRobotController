package org.firstinspires.ftc.teamcode.subsystems.VisionPipelines;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class GreenDetectPipeline extends OpenCvPipeline {
    public static int m_stageToShow = 0;

    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    Mat mask = new Mat();
    Mat erodedMask = new Mat();
    Mat input_hsv = new Mat();
    Mat input_bgr = new Mat();
    Mat cropped = new Mat();
    Mat input_withAngle = new Mat();
    Scalar sumValue = new Scalar(0);
    float totalPixs = 0.0f;
    float[] sumValNorm = new float[3];
    List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    Mat m_hierarchy = new Mat();
    Mat input_withContours = new Mat();
    Mat input_withBB = new Mat();
    int maxContourIdx = 0;
    double maxVal = 0.0;
    double centerX = 0.0;
    double centerY = 0.0;
    double pixToAngleFactor = 0.0703125;
    double angleX = 0.0;
    int imageSizeX = 640;
    int imageSizeY = 480;

    public static double H_start = 60.0;
    public static double H_end = 80.0;



    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5,5), new Point(-1,-1));


    @Override
    public Mat processFrame(Mat input)
    {
        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */
//            USE LOWERlim ~60-70, UPPERLIM ~80-90 for green detection
        Imgproc.cvtColor(input, input_bgr,Imgproc.COLOR_RGBA2BGR); //EasyOpenCV return images in RGBA format
        Imgproc.cvtColor(input_bgr, input_hsv, Imgproc.COLOR_BGR2HSV); // We convert them to BGR since only BGR (or RGB) conversions to HSV exist
        Core.inRange(input_hsv,
                new Scalar(H_start,50,50),
                new Scalar(H_end,255,255),
                mask);

        //Erode/Dialate the image to get rid of small pixels
        Imgproc.erode(mask, erodedMask, kernel);


        //Draw contours around detected pixels
        contours.clear();
        Imgproc.findContours(erodedMask, contours, m_hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE );

        maxVal = 0.0;
        maxContourIdx = 0;
        for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++)
        {
            double contourArea = Imgproc.contourArea(contours.get(contourIdx));
            if (maxVal < contourArea)
            {
                maxVal = contourArea;
                maxContourIdx = contourIdx;
            }
        }

        input.copyTo(input_withContours);

        if(contours.size() >= 1) {
            Imgproc.drawContours(input_withContours, contours, maxContourIdx, new Scalar(255), 5);
            Rect boundingBox = Imgproc.boundingRect(contours.get(maxContourIdx));

            input.copyTo(input_withBB);
            Imgproc.rectangle(input_withBB, boundingBox, new Scalar(300));

            centerX = boundingBox.x + boundingBox.width / 2.0;
            centerY = boundingBox.y + boundingBox.height / 2.0;
        }
        else
        {
            centerX = imageSizeX/2.0;
            centerY = imageSizeY/2.0;
        }

        angleX = (centerX - imageSizeX/2.0) * pixToAngleFactor;

        input.copyTo(input_withAngle);
        Imgproc.putText(input_withAngle, Double.toString(angleX), new Point(50,50), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255));
        Imgproc.circle(input_withAngle, new Point(centerX, centerY), 4, new Scalar(255), 5);
        switch(m_stageToShow) {
            case 1:
                return mask;
            case 2:
                return erodedMask;
            case 3:
                return input_withContours;
            case 4:
                return input_withBB;
            case 5:
                return input_withAngle;
            default:
                return input;
        }

//45.22 degrees horiontal feild of view
//55 degrees diagonal FOV
        //conversion factor = 45/640
    }

    public double getAngle()
    {
        return angleX;
    }

    public void setStageToShow(int stageIndex)
    {
        m_stageToShow = stageIndex;
    }
}
