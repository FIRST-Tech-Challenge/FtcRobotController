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
public class RedDetectPipeline extends OpenCvPipeline {
    public static int m_stageToShow = 0;

    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    //Image processing buffers
    Mat mask = new Mat();
    Mat erodedMask = new Mat();
    Mat input_hsv = new Mat();
    Mat input_bgr = new Mat();
    Mat cropped = new Mat();
    Mat erosionKernel = new Mat();
    Mat input_withBoxes = new Mat();

    Scalar sumValue = new Scalar(0);
    float totalPixs = 0.0f;
    float[] sumValNorm = new float[2];
    double maxVal = 0.0;
    int maxIndex = -1;

    public static double object = 0.001;

    //Color thresholding (Hue) upper/lower bounds
    public static double H_start = 175;
    public static double H_end = 190.0;

    //Erosion Kernel
    public static double kernelDimx = 7;
    public static double kernelDimy= 7;

    //detected position text location x,y
    public static int textLocX = 50;
    public static int textLocY = 250;
    public static int textScale = 3;
    public static int textThickness = 12;

    public Boolean isFinished = false;

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
        //Color Thresholding -- use LOWERlim ~60-70, UPPERLIM ~80-90 for green detection
        Imgproc.cvtColor(input, input_bgr,Imgproc.COLOR_RGBA2BGR); //EasyOpenCV return images in RGBA format
        Imgproc.cvtColor(input_bgr, input_hsv, Imgproc.COLOR_BGR2HSV); // We convert them to BGR since only BGR (or RGB) conversions to HSV exist
        Core.inRange(input_hsv,
                new Scalar(H_start,50,50),
                new Scalar(H_end,255,255),
                mask);

        //Erode (and possibly dilate) the image to get rid of small pixels
        erosionKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(kernelDimx,kernelDimy), new Point(-1,-1));
        Imgproc.erode(mask, erodedMask, erosionKernel);
        Imgproc.dilate(erodedMask, erodedMask, erosionKernel);

        //This is the input copy image used where we draw the cropping boxes on it
        input.copyTo(input_withBoxes);

        //Draw three simple boxes around the middle of the image
        maxVal = 0.0;
        // Initialize variables to store information about the boxes
        int maxIndex = -1;
        float maxVal = -1.0f;
        float objectAmount = (float) object;
        for (int i = 0; i < 2; i++) {
            // Draw rectangles on the input_withBoxes image
            Imgproc.rectangle(
                    input_withBoxes,
                    new Point(i * input.cols() / 2, 0),
                    new Point((i + 1) * input.cols() / 2, input.rows()),
                    new Scalar(0, 0, 255), 4);

            // Extract submatrices from the erodedMask
            cropped = erodedMask.submat(
                    new org.opencv.core.Range(0, input.rows()),
                    new org.opencv.core.Range(i * input.cols() / 2, (i + 1) * input.cols() / 2)
            );

            // Sum all the color thresholded pixels to detect the color of interest
            sumValue = Core.sumElems(cropped);
            totalPixs = (float) (cropped.cols() * cropped.rows());
            sumValNorm[i] = (float) (sumValue.val[0]) / totalPixs / 255.0f; // Check if scaling is correct

            // Get the index of the box with the most color-thresholded pixels
            if (sumValNorm[i] > maxVal) {
                maxVal = sumValNorm[i];
                maxIndex = i;
            }

        }
        if (maxVal < objectAmount) {
            maxIndex = 2;
        }
        isFinished = true;

// maxIndex now contains the index of the box with the most of a certain pixel
// You can use this information as needed for further processing



        //Write the detected position on the image
        Imgproc.putText(input_withBoxes, Integer.toString(maxIndex), new Point(textLocX,textLocY), Imgproc.FONT_HERSHEY_SIMPLEX, textScale, new Scalar(0,0,255), textThickness);
        switch(m_stageToShow) {
            case 1:
                return mask;
            case 2:
                return erodedMask;
            case 3:
                return input_withBoxes;
            default:
                return input;
        }

        //45.22 degrees horiontal feild of view
        //55 degrees diagonal FOV
        //conversion factor = 45/640
    }

    public int getLocation()
    {
        return maxIndex;
    }

}