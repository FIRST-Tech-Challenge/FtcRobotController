package org.firstinspires.ftc.teamcode.TestOpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.opencv.core.*;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CircleDetection extends OpenCvPipeline
{
    Mat grayMat = new Mat();
    Mat blurMat = new Mat();
    Mat hsvMaskedMat = new Mat();
    Mat circlesOnFrameMat = new Mat();
    int numCirclesFound;

    enum Stage
    {
        GREY,
        BLUR,
        MASKED,
        CIRCLES_OVERLAYED_ON_FRAME,
        RAW_IMAGE,
    }

    private Stage stageToRenderToViewport = Stage.CIRCLES_OVERLAYED_ON_FRAME;
    private Stage[] stages = Stage.values();

    private Telemetry telemetry;

    public CircleDetection  (Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void onViewportTapped()
    {
        int currentStageNum = stageToRenderToViewport.ordinal();

        int nextStageNum = currentStageNum + 1;

        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }

        stageToRenderToViewport = stages[nextStageNum];
    }

    @Override
    public Mat processFrame(Mat input)
    {
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Mat mask = new Mat();
        Mat hsvMat = new Mat();

        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsvMat, new Scalar(0, 70, 50), new Scalar(10, 255, 255), mask1);
        Core.inRange(hsvMat, new Scalar(160, 70, 50), new Scalar(180, 255, 255), mask2);

        Core.bitwise_or(mask1, mask2, mask);
        hsvMaskedMat.release();
        Core.bitwise_and(input, input, hsvMaskedMat, mask);

        Imgproc.cvtColor(hsvMaskedMat, grayMat, Imgproc.COLOR_RGB2GRAY);

        Imgproc.GaussianBlur(grayMat, blurMat, new org.opencv.core.Size(15.0, 15.0), 2, 2);
        Mat circles = new Mat();
        Imgproc.HoughCircles(blurMat, circles, Imgproc.HOUGH_GRADIENT, 1, 150, 130, 30);
        numCirclesFound = circles.cols();
        input.copyTo(circlesOnFrameMat);
        for(int i=0; i < numCirclesFound; i++)
        {
            double[] data = circles.get(0, i);
            Point center = new Point(Math.round(data[0]), Math.round(data[1]));
            // circle center
            Imgproc.circle(circlesOnFrameMat, center, 1, new Scalar(0, 0, 255), 2, 8, 0 );
            // circle outline
            int radius = (int) Math.round(data[2]);
            Imgproc.circle(circlesOnFrameMat, center, radius, new Scalar(0,0,255), 2, 8, 0 );
        }

        telemetry.addData("[Stage]", stageToRenderToViewport);
        telemetry.addData("[Found Circles]", "%d", numCirclesFound);
        telemetry.update();

        switch (stageToRenderToViewport)
        {
            case GREY:
            {
                return grayMat;
            }

            case MASKED:
            {
                return hsvMaskedMat;
            }

            case BLUR:
            {
                return blurMat;
            }

            case CIRCLES_OVERLAYED_ON_FRAME:
            {
                return circlesOnFrameMat;
            }

            case RAW_IMAGE:
            {
                return input;
            }

            default:
            {
                return input;
            }
        }
    }
}
