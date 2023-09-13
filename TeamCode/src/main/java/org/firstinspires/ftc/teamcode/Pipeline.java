package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.core.Scalar;
//import org.opencv.core.vec3b;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Rect;

public class Pipeline extends OpenCvPipeline {
    Mat mat = new Mat();
    Mat red1mask = new Mat();
    Mat red2mask = new Mat();
    Mat redmask = new Mat();
    Mat greenmask = new Mat();
    Mat bluemask = new Mat();

    public static String colorString;

    private static double colorScale = 0.75;

    // Color Evaluation
    //pink = 240, 91, 146 RGB
    Scalar red1lowColorValue = new Scalar(0,70,50);
    Scalar red1highColorValue = new Scalar(20,255,255);
    Scalar red2lowColorValue = new Scalar(160,70,50);
    Scalar red2highColorValue = new Scalar(180,255,255);
    //Scalar pinklowColorValue = pinkValue.mul(pinkValue,1-colorScale);
    //Scalar pinkhighColorValue = pinkValue.mul(pinkValue,1+colorScale);

    //green = 183, 203, 176 RGB
    Scalar greenlowColorValue = new Scalar(40,100,50);
    Scalar greenhighColorValue = new Scalar(80,255,255);
    //Scalar greenValue = new Scalar(189,8,61);
    //Scalar greenlowColorValue = greenValue.mul(greenValue,1-colorScale);
    //Scalar greenhighColorValue = greenValue.mul(greenValue,1+colorScale);

    //purple = 182, 152, 237 RGB
    Scalar bluelowColorValue = new Scalar(100,150,0);
    Scalar bluehighColorValue = new Scalar(140,255,255);
    //Scalar purpleValue = new Scalar(255,28,61);
    //Scalar purplelowColorValue = purpleValue.mul(purpleValue,1-colorScale);
    //Scalar purplehighColorValue = purpleValue.mul(purpleValue,1+colorScale);

    /*private int LeftROIStartRow = 180;
    private int LeftROIEndRow = 320;
    private int MiddleROIStartRow = 20;
    private int MiddleROIEndRow = 150;
    private int ROIStartCol = 60; // Expanded height in case camera tilts
    private int ROIEndCol = 180; // Expanded height in case camera tilts
    */
    private int resultROI;
    private double[] centerPix;

    @Override
    public Mat processFrame(Mat input)
    {
        Rect rectCrop = new Rect(160,60,80,120);
        //Mat cropped_image = input(rectCrop);
        Mat image_cropped = new Mat(input,rectCrop);
        //Imgproc.cvtColor(cropped_image, mat, Imgproc.COLOR_BGR2RGBA);
        //Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(image_cropped,mat, Imgproc.COLOR_RGB2HSV);
        Size sizeInput = image_cropped.size();
        int height = (int)sizeInput.height;
        int width = (int)sizeInput.width;
        centerPix = image_cropped.get((height/2),(width/2));

        // Creates mask to identify specific color

        // Applies mask.  Most colors become black, some become white.
        Core.inRange(mat, red1lowColorValue,red1highColorValue,red1mask);
        Core.inRange(mat,red2lowColorValue,red2highColorValue,red2mask);
        Core.inRange(mat, greenlowColorValue, greenhighColorValue, greenmask);
        Core.inRange(mat, bluelowColorValue, bluehighColorValue, bluemask);
        Core.bitwise_or(red1mask,red2mask,redmask);

        // Create the areas we are interested in looking at.
        //Mat LeftROI = mat.submat(LeftROIStartRow, LeftROIEndRow, ROIStartCol, ROIEndCol);
        //Mat MiddleROI = mat.submat(MiddleROIStartRow, MiddleROIEndRow, ROIStartCol, ROIEndCol);

        //setResultROI(evaluateROIs(LeftROI, MiddleROI));
        setResultROI(evaluateROIs(redmask,greenmask,bluemask));

        //LeftROI.release();      // Added by Ohm Raiders to prevent memory leak
        //MiddleROI.release();    // Added by Ohm Raiders

        // Adds the rectangles so we can see where we are looking (the ROIs)
        //Imgproc.rectangle(mat, new Point(ROIStartCol, LeftROIStartRow), new Point(ROIEndCol, LeftROIEndRow), new Scalar(128,128,128), 2);
        //Imgproc.rectangle(mat, new Point(ROIStartCol, MiddleROIStartRow), new Point(ROIEndCol, MiddleROIEndRow), new Scalar(128,128,128), 2);
        return input;
    }

    private int evaluateROIs(Mat redmask, Mat greenmask, Mat bluemask) {
        // Returns results:
        // 0 - Left
        // 1 - Middle
        // 2 - Right
        int redResult = findWhiteCount(redmask);
        int greenResult = findWhiteCount(greenmask);
        int blueResult = findWhiteCount(bluemask);

        if (redResult>blueResult && redResult>greenResult)
            return 1;
        else if (greenResult>redResult && greenResult > blueResult)
            return 2;
        else if (blueResult>redResult && blueResult > greenResult)
            return 3;
        else
            return 0;
    }

    private int findWhiteCount(Mat roi) {
        int count = 0;
        for (int row = 0; row < roi.height(); row++) {
            for (int col = 0; col < roi.width(); col++) {
                if (roi.get(row, col)[0] > 1) {
                    count += 1;
                }
            }
        }
        return count;
    }

    public int getResultROI()
    {
        return resultROI;
    }
    public double [] getCenterPix() {return centerPix;}

    private void setResultROI(int roi)
    {
        resultROI = roi;
    }
}
