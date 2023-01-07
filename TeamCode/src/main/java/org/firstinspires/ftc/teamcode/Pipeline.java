package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {
    Mat mat = new Mat();
    Mat pinkmat = new Mat();
    Mat greenmat = new Mat();
    Mat purplemat = new Mat();
    // Color Evaluation
    Scalar pinklowColorValue = new Scalar(136, 81, 230);
    Scalar pinkhighColorValue = new Scalar(156, 101, 250);

    Scalar greenlowColorValue = new Scalar(166, 193, 173);
    Scalar greenhighColorValue = new Scalar(186, 213, 193);

    Scalar purplelowColorValue = new Scalar(227, 142, 172);
    Scalar purplehighColorValue = new Scalar(247, 162, 192);

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
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2RGBA);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGBA2BGR);

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
        return pinkmat;
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
