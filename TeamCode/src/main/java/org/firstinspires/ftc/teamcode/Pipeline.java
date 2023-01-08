package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {
    Mat mat = new Mat();
    Mat red1mask = new Mat();
    Mat red2mask = new Mat();
    Mat redmask = new Mat();
    Mat greenmask = new Mat();
    Mat bluemask = new Mat();

    private static double colorScale = 0.75;

    // Color Evaluation
    //pink = 240, 91, 146 RGB
    Scalar red1lowColorValue = new Scalar(0,70,50);
    Scalar red1highColorValue = new Scalar(10,255,255);
    Scalar red2lowColorValue = new Scalar(170,70,50);
    Scalar red2highColorValue = new Scalar(180,255,255);
    //Scalar pinklowColorValue = pinkValue.mul(pinkValue,1-colorScale);
    //Scalar pinkhighColorValue = pinkValue.mul(pinkValue,1+colorScale);

    //green = 183, 203, 176 RGB
    Scalar greenlowColorValue = new Scalar(55,100,100);
    Scalar greenhighColorValue = new Scalar(65,255,255);
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

    @Override
    public Mat processFrame(Mat input)
    {
        //Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2RGBA);
        //Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(input,mat, Imgproc.COLOR_BGR2HSV);
        Size sizeInput = input.size();
        int height = (int)sizeInput.height;
        int width = (int)sizeInput.width;
        for (int i = width/2-5; i < width/2+5; i = i+1)
        {
            for (int j = height/2-5; j < height/2+5; j = j+1)
            {
                double [] pix = input.get(i, j);
                telemetry.addData(String.format("(%d,%d) = (%f,%f,%f)\n", i, j, pix[0], pix[1], pix[2]),"");
            }
        }
        telemetry.update();
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
        return mat;
    }

    private int evaluateROIs(Mat redmask, Mat greenmask, Mat bluemask) {
        // Returns results:
        // 0 - Left
        // 1 - Middle
        // 2 - Right
        int redResult = findWhiteCount(redmask);
        int greenResult = findWhiteCount(greenmask);
        int blueResult = findWhiteCount(bluemask);

        if (blueResult<50 && redResult<50)
        {
            return 2;

        }
        else if (blueResult>redResult)
        {
            return 3;

        }
        else if (redResult>blueResult)
            return 1;
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

    private void setResultROI(int roi)
    {
        resultROI = roi;
    }
}
