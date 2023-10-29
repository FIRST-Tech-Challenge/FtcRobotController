package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline2023 extends OpenCvPipeline {
    Mat mat = new Mat();
    Mat red1mask = new Mat();
    Mat red2mask = new Mat();
    Mat mask = new Mat();

    public static String colorString;

    private static double colorScale = 0.75;

    // Color Evaluation

    Scalar red1lowColorValue = new Scalar(0,70,50);
    Scalar red1highColorValue = new Scalar(20,255,255);
    Scalar red2lowColorValue = new Scalar(160,70,50);
    Scalar red2highColorValue = new Scalar(180,255,255);


    Scalar bluelowColorValue = new Scalar(100,150,0);
    Scalar bluehighColorValue = new Scalar(140,255,255);


    private int resultROI;
    private double[] centerPix;
    private boolean useBlue;

    public Pipeline2023(boolean useBlueArg){
        super();
        useBlue = useBlueArg;
        resultROI = 3;
    }


    @Override
    public Mat processFrame(Mat input)
    {
        Rect rectCrop = new Rect(160,60,80,120);
        //Mat cropped_image = input(rectCrop);
        Mat image_cropped = (input);
        //Imgproc.cvtColor(cropped_image, mat, Imgproc.COLOR_BGR2RGBA);
        //Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(image_cropped,mat, Imgproc.COLOR_RGB2HSV);
        Size sizeInput = image_cropped.size();
        int height = (int)sizeInput.height;
        int width = (int)sizeInput.width;
        centerPix = image_cropped.get((height/2),(width/2));

        // Creates mask to identify specific color

        // Applies mask.  Most colors become black, some become white.
        if (useBlue) {
            Core.inRange(mat, bluelowColorValue, bluehighColorValue, mask);
        }
        else {
            Core.inRange(mat, red1lowColorValue,red1highColorValue,red1mask);
            Core.inRange(mat,red2lowColorValue,red2highColorValue,red2mask);
            Core.bitwise_or(red1mask,red2mask, mask);
        }
        // Create the areas we are interested in looking at.
        //Mat LeftROI = mat.submat(LeftROIStartRow, LeftROIEndRow, ROIStartCol, ROIEndCol);
        //Mat MiddleROI = mat.submat(MiddleROIStartRow, MiddleROIEndRow, ROIStartCol, ROIEndCol);

        //setResultROI(evaluateROIs(LeftROI, MiddleROI));
        setResultROI(evaluateROIs(mask));

        //LeftROI.release();      // Added by Ohm Raiders to prevent memory leak
        //MiddleROI.release();    // Added by Ohm Raiders

        // Adds the rectangles so we can see where we are looking (the ROIs)
        //Imgproc.rectangle(mat, new Point(ROIStartCol, LeftROIStartRow), new Point(ROIEndCol, LeftROIEndRow), new Scalar(128,128,128), 2);
        //Imgproc.rectangle(mat, new Point(ROIStartCol, MiddleROIStartRow), new Point(ROIEndCol, MiddleROIEndRow), new Scalar(128,128,128), 2);
        return mask;
    }

    private int evaluateROIs(Mat mask) {
        // Returns results:
        // 0 - Left
        // 1 - Middle
        // 2 - Right
        int leftResult = findWhiteCount(mask,0,mask.width()/3);
        int middleResult = findWhiteCount(mask, mask.width()/3, mask.width()/3);
        int rightResult = findWhiteCount(mask, mask.width()*2/3, mask.width()/3);

        if ((leftResult > middleResult) && (leftResult > rightResult)) {
            return 0;
        }

        else if  ((rightResult > leftResult) && (rightResult > middleResult)) {
            return 2;
        }
        else {
            return 1;
        }
    }

    private int findWhiteCount(Mat roi,int start,int width) {
        int count = 0;
        for (int row = 0; row < roi.height(); row++) {
            for (int col = start; col < start+width; col++) {
                if (roi.get(row, col)[0] >= 1) {
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
