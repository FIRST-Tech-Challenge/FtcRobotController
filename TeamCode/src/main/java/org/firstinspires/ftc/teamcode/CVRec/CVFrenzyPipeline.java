package org.firstinspires.ftc.teamcode.CVRec;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

public class CVFrenzyPipeline extends CVPipelineBase {


    private Mat region_Cb;
    private Mat region2_Cb;
    private Mat YCrCb = new Mat();
    private Mat Cb = new Mat();

    static final Point REGION_ANCHOR_POINT = new Point(40,100);
    static final int REGION_WIDTH = 70;
    static final int REGION_HEIGHT = 100;

    static final Point REGION_ANCHOR_POINT_2 = new Point(210,100);
    static final int REGION_WIDTH_2 = 70;
    static final int REGION_HEIGHT_2 = 100;

    Point region_pointA = new Point(
            REGION_ANCHOR_POINT.x,
            REGION_ANCHOR_POINT.y);
    Point region_pointB = new Point(
            REGION_ANCHOR_POINT.x + REGION_WIDTH,
            REGION_ANCHOR_POINT.y + REGION_HEIGHT);

    Point region_pointA_2 = new Point(
            REGION_ANCHOR_POINT_2.x,
            REGION_ANCHOR_POINT_2.y);
    Point region_pointB_2 = new Point(
            REGION_ANCHOR_POINT_2.x + REGION_WIDTH_2,
            REGION_ANCHOR_POINT_2.y + REGION_HEIGHT_2);

    public CVFrenzyPipeline(int resX, int resY){
        super(resX, resY);
    }

    @Override
    public void init(Mat mat) {
        inputToCb(mat);

        region_Cb = Cb.submat(new Rect(region_pointA, region_pointB));
        region2_Cb = Cb.submat(new Rect(region_pointA_2, region_pointB_2));
    }



    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);
        meanVal = (int) Core.mean(region_Cb).val[0];
        meanVal_2 = (int) Core.mean(region2_Cb).val[0];

        //Draw rectangle
        Imgproc.rectangle(
                input, // Buffer to draw on
                region_pointA, // First point which defines the rectangle
                region_pointB, // Second point which defines the rectangle
                CVDetector.BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines
        Imgproc.rectangle(
                input, // Buffer to draw on
                region_pointA_2, // First point which defines the rectangle
                region_pointB_2, // Second point which defines the rectangle
                CVDetector.BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        if (getMeanVal() < ORANGE){
            setGameElement(GameElement.CubeLocation1);
        }
        else if (getMeanVal_2() < ORANGE){
            setGameElement(GameElement.CubeLocation2);
        }
        else {
            setGameElement(GameElement.CubeLocationNone);
        }
        return input;
    }


    private void inputToCb(Mat input) {
        //convert the input matrix color space from RGB to YCrCb.
        // Because the way YCrCb represents color by luminance(Y), chroma of red(CR), chroma of blue(Cb),
        // it keeps values consistent under different lighting
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        //Extracts the blue channel in Cb variable
        Core.extractChannel(YCrCb, Cb, 2);
    }
}
