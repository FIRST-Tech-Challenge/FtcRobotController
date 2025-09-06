package org.firstinspires.ftc.teamcode.VisionPipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class WebcamPipelineNotes extends OpenCvPipeline {

    //initializing Mats outside of functions are better because you avoid leaking memory
    Mat subMat;
    Mat outputMat;
    Mat REDcolormap = new Mat();
    Mat filteredLeft;
    Mat filteredRight;

    double LEFTAVG;
    double RIGHTAVG;

    Scalar RED = new Scalar(255.0, 0.0, 0.0);

    Telemetry telemetry;

    //This happens when you call the pipeline with the frame
    @Override
    public void init(Mat firstFrame)
    {
        // subMats are a small portion of the mat in this case you make a subMat of
        // starting from x:0 to x:500, and y:0 to y:500
    }

    // you process the frame
    @Override
    public Mat processFrame(Mat input)
    {
        // boilerplate code all it does is tell you which side a red object is on.
        //normally image processing in the processFrame function.
        Imgproc.cvtColor(input, REDcolormap, Imgproc.COLOR_RGB2YCrCb);

        // create rectangles to split the screen in half
        Rect leftRect = new Rect(1, 1, 319, 359);
        Rect rightRect = new Rect(320, 1, 319, 359);

        //set outputMat as a separate clone of the input frame
        outputMat = input.clone();
        //draw the rectangle to the screen
        Imgproc.rectangle(outputMat, leftRect, RED, 2);
        Imgproc.rectangle(outputMat, rightRect, RED, 2);

        //we making a subMat here
        filteredLeft = REDcolormap.submat(leftRect);
        filteredRight = REDcolormap.submat(rightRect);

        // extract the channel of index 1 and copy it into the destination mat, which is the same mat in this case.
        // the order of extractChannel is Source, Destination, and Channel.
        Core.extractChannel(filteredLeft, filteredLeft, 2);
        Core.extractChannel(filteredRight, filteredRight, 2);

        // find the mean of all the rgb values and store the averages or R, G, and B into a single scalar value
        Scalar leftAvg = Core.mean(filteredLeft);
        Scalar rightAvg = Core.mean(filteredRight);

        // you can't just do leftAvg[0] for some reason, but you can get the R value by doing .val[0]
        LEFTAVG = leftAvg.val[0];
        RIGHTAVG = rightAvg.val[0];

        // Simple comparisons.
        if (LEFTAVG > RIGHTAVG)
        {
            telemetry.addData("[Cool!]","Left");
        }
        else
        {
            telemetry.addData("This is amazing", "Right");
        }


        return outputMat;
    }

    // call WebcamPipelineNotes.results() to get results of your pipeline
    public int currentresult(int results)
    {
        return results;
    }
}

