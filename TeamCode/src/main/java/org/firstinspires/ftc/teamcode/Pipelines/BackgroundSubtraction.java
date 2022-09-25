package org.firstinspires.ftc.teamcode.Pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;
import org.opencv.video.BackgroundSubtractor;
import org.opencv.video.Video;

/**
 * Remove the background given an input matrix.
 * Uses monovision motion elimination to remove the background
 * so only things that move will be considered in the foreground.
 */
public class BackgroundSubtraction extends OpenCvPipeline {
    /**
     * Telemetry object to display data to the console
     */
    private Telemetry telemetry = null;

    /**
     * A background subtractor object using MOG2 to remove the background in an input image.
     */
    private BackgroundSubtractor backSub = Video.createBackgroundSubtractorMOG2();


    /**
     * Constructor to assign the telemetry object and actually have telemetry work.
     * This works because the OpenCvPipeline object has this built in interface to
     * use telemetry.
     * @param telemetry the input Telemetry type object.
     */
    public BackgroundSubtraction(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Process an input frame from the webcamera and remove the background in it.
     * @param input is the input matrix from the webcam
     * @return the output matrix consisting of a removed background.
     */
    @Override
    public Mat processFrame(Mat input){
        telemetry.addData("processFrame(...) running!","");
        telemetry.update();

        Mat fgMask = new Mat();
        backSub.apply(input, fgMask);

        input.release();

        return fgMask;
    }

}