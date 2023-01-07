package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {
    Mat mat = new Mat();
    Mat pinkmat = new Mat();
    Mat greenmat = new Mat();
    Mat purplemat = new Mat();

    private static double colorScale = 0.75;
    private static int hDelta = 10;
    private static int sDelta = 8;
    private static int vDelta = 40;

    // Color Evaluation
    //pink = 240, 91, 146 RGB
    private static int pinkH = 335;
    private static int pinkS = 47;
    private static int pinkV = 62;//= new Scalar(335,47,62);
    Scalar pinklowColorValue = new Scalar(pinkH-hDelta, pinkS-sDelta, pinkV-vDelta);
    Scalar pinkhighColorValue = new Scalar(pinkH+hDelta, pinkS+sDelta, pinkV+vDelta);
    //Scalar pinklowColorValue = pinkValue.mul(pinkValue,1-colorScale);
    //Scalar pinkhighColorValue = pinkValue.mul(pinkValue,1+colorScale);

    //green = 183, 203, 176 RGB
    private static int greenH = 189;
    private static int greenS = 8;
    private static int greenV = 61;
    Scalar greenlowColorValue = new Scalar(greenH-hDelta, greenS-sDelta, greenV-vDelta);
    Scalar greenhighColorValue = new Scalar(greenH+hDelta, greenS+sDelta, greenV+vDelta);
    //Scalar greenValue = new Scalar(189,8,61);
    //Scalar greenlowColorValue = greenValue.mul(greenValue,1-colorScale);
    //Scalar greenhighColorValue = greenValue.mul(greenValue,1+colorScale);

    //purple = 182, 152, 237 RGB
    private static int purpleH = 255;
    private static int purpleS = 28;
    private static int purpleV = 61;
    Scalar purplelowColorValue = new Scalar(purpleH-hDelta, purpleS-sDelta, purpleV-vDelta);
    Scalar purplehighColorValue = new Scalar(purpleH+hDelta, purpleS+sDelta, purpleV+vDelta);
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
        Core.inRange(mat, pinklowColorValue, pinkhighColorValue, pinkmat);
        Core.inRange(mat, greenlowColorValue, greenhighColorValue, greenmat);
        Core.inRange(mat, purplelowColorValue, purplehighColorValue, purplemat);


        // Create the areas we are interested in looking at.
        //Mat LeftROI = mat.submat(LeftROIStartRow, LeftROIEndRow, ROIStartCol, ROIEndCol);
        //Mat MiddleROI = mat.submat(MiddleROIStartRow, MiddleROIEndRow, ROIStartCol, ROIEndCol);

        //setResultROI(evaluateROIs(LeftROI, MiddleROI));
        setResultROI(evaluateROIs(pinkmat, greenmat, purplemat));

        //LeftROI.release();      // Added by Ohm Raiders to prevent memory leak
        //MiddleROI.release();    // Added by Ohm Raiders

        // Adds the rectangles so we can see where we are looking (the ROIs)
        //Imgproc.rectangle(mat, new Point(ROIStartCol, LeftROIStartRow), new Point(ROIEndCol, LeftROIEndRow), new Scalar(128,128,128), 2);
        //Imgproc.rectangle(mat, new Point(ROIStartCol, MiddleROIStartRow), new Point(ROIEndCol, MiddleROIEndRow), new Scalar(128,128,128), 2);
        return purplemat;
    }

    private int evaluateROIs(Mat pinkmat, Mat greenmat, Mat purplemat) {
        // Returns results:
        // 0 - Left
        // 1 - Middle
        // 2 - Right
        int pinkResult = findWhiteCount(pinkmat);
        int greenResult = findWhiteCount(greenmat);
        int purpleResult = findWhiteCount(purplemat);

        if (purpleResult<50 && pinkResult<50)
        {
            return 2;

        }
        else if (purpleResult>pinkResult)
        {
            return 3;

        }
        else if (pinkResult>purpleResult)
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
