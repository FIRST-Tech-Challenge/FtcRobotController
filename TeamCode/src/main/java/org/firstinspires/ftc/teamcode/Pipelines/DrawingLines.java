package org.firstinspires.ftc.teamcode.Pipelines;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * Class to draw two lines on an input image.
 */
public class DrawingLines extends OpenCvPipeline{

    /**
     * Take an input image as a matrix and draw two lines on it.
     * @param input The input image
     * @return the input image with two lines drawn on it to be displayed on the console
     */
    @Override
    public Mat processFrame(Mat input){
        // Frame height: 176 pixels, Frame width: 144 pixels for THIS specific camera - input.cols()/input.rows() is MUCH safer to use!

        // Draw a diagonal line across the screen
        Imgproc.line(
                input,                                  // Input matrix to draw a line on
                new Point(0, 0),                   // First point on the line - (0,0) is the TOP LEFT of the frame.
                new Point(input.cols(), input.rows()),  // Second point on the line - this is the bottom right corner
                new Scalar(100, 100, 100),              // Line color - RGB scalar 255 max
                1,                              // Line thickness
                Imgproc.LINE_4,                         // Line format - DO NOT CHANGE
                0);                                // Line format - DO NOT CHANGE


        // Draw a horizontal line 5/6ths of the way down the line.
        Imgproc.line(
                input,                                             // Input matrix to draw a line on
                new Point(0, input.rows()*(5.0/6)),          // First point on the line - (0,0) is the TOP LEFT of the frame.
                new Point(input.cols(), input.rows()*(5.0/6)),  // Second point on the line - this is the bottom right corner
                new Scalar(255, 50, 100),                          // Line color - RGB scalar 255 max
                3,                                         // Line thickness
                Imgproc.LINE_4,                                    // Line format - DO NOT CHANGE
                0);                                           // Line format - DO NOT CHANGE

        return input;
    }
}
