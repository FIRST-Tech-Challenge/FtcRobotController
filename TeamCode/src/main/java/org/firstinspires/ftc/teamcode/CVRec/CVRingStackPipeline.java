package org.firstinspires.ftc.teamcode.CVRec;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

public class CVRingStackPipeline extends CVPipelineBase {

    private Mat region_Cb;
    private Mat YCrCb = new Mat();
    private Mat Cb = new Mat();


    static final Point REGION_ANCHOR_POINT = new Point(150,100);
    static final int REGION_WIDTH = 60;
    static final int REGION_HEIGHT = 60;

    Point region_pointA = new Point(
            REGION_ANCHOR_POINT.x,
            REGION_ANCHOR_POINT.y);
    Point region_pointB = new Point(
            REGION_ANCHOR_POINT.x + REGION_WIDTH,
            REGION_ANCHOR_POINT.y + REGION_HEIGHT);

    public CVRingStackPipeline(int resX, int resY){
        super(resX, resY);
    }

    @Override
    public void init(Mat mat) {
        inputToCb(mat);

        region_Cb = Cb.submat(new Rect(region_pointA, region_pointB));
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);

        meanVal = (int) Core.mean(region_Cb).val[0];

        //Draw rectangle
        Imgproc.rectangle(
                input, // Buffer to draw on
                region_pointA, // First point which defines the rectangle
                region_pointB, // Second point which defines the rectangle
                CVDetector.BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        if (getMeanVal() < QUAD_MAX){
            stackSize = RingStackSize.Quad;
        }
        else if (getMeanVal() < SINGLE_MAX){
            stackSize = RingStackSize.Single;
        }
        else{
            stackSize = RingStackSize.None;
        }


        return input;
    }

    private void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }
}
