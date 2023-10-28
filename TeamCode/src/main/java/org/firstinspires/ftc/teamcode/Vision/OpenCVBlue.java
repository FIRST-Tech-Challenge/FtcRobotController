/*
 * Copyright (c) 2021 Sebastian Erives
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//@I2cDeviceType()
//@TeleOp
public class OpenCVBlue extends OpenCvPipeline {

    private volatile String result = "start";

    /*
     * These are our variables that will be
     * modifiable from the variable tuner.
     *
     * Scalars in OpenCV are generally used to
     * represent color. So our values in the
     * lower and upper Scalars here represent
     * the Y, Cr and Cb values respectively.
     *
     * YCbCr, like most color spaces, range
     * from 0-255, so we default to those
     * min and max values here for now, meaning
     * that all pixels will be shown.
     */
    //public String result = "";
    private Scalar lower = new Scalar(96.3f, 73.7f, 0);
    private Scalar upper = new Scalar(147.3f, 255, 199.8f);
    /**
     * This will allow us to choose the color
     * space we want to use on the live field
     * tuner instead of hardcoding it
     */
    private ColorSpace colorSpace = ColorSpace.HSV;

    /*
     * A good practice when typing EOCV pipelines is
     * declaring the Mats you will use here at the top
     * of your pipeline, to reuse the same buffers every
     * time. This removes the need to call mat.release()
     * with every Mat you create on the processFrame method,
     * and therefore, reducing the possibility of getting a
     * memory leak and causing the app to crash due to an
     * "Out of Memory" error.
     */
    private Mat ycrcbMat       = new Mat();
    private Mat binaryMat      = new Mat();
    public Mat maskedInputMat = new Mat();

    Mat dest_matrix = new Mat();
    Mat stats_mat = new Mat();
    Mat cens_mat = new Mat();

    private Telemetry telemetry = null;

    /**
     * Enum to choose which color space to choose
     * with the live variable tuner isntead of
     * hardcoding it.
     */
    enum ColorSpace {
        /*
         * Define our "conversion codes" in the enum
         * so that we don't have to do a switch
         * statement in the processFrame method.
         */
        RGB(Imgproc.COLOR_RGBA2RGB),
        HSV(Imgproc.COLOR_RGB2HSV),
        YCrCb(Imgproc.COLOR_RGB2YCrCb),
        Lab(Imgproc.COLOR_RGB2Lab);



        //store cvtCode in a public var
        public int cvtCode = 0;

        //constructor to be used by enum declarations above
        ColorSpace(int cvtCode) {
            this.cvtCode = cvtCode;
        }
    }

    public OpenCVBlue(Telemetry telemetry) {
        this.telemetry = telemetry;
    }




    public String getResult()
    {
        return result;
    }

    public Mat processFrame(Mat input) {
        /*
         * Converts our input mat from RGB to
         * specified color space by the enum.
         * EOCV ALWAYS returns RGB mats, so you'd
         * always convert from RGB to the color
         * space you want to use.
         *
         * Takes our "input" mat as an input, and outputs
         * to a separate Mat buffer "ycrcbMat"
         */
        Imgproc.cvtColor(input, ycrcbMat, colorSpace.cvtCode);

        /*
         * This is where our thresholding actually happens.
         * Takes our "ycrcbMat" as input and outputs a "binary"
         * Mat to "binaryMat" of the same size as our input.
         * "Discards" all the pixels outside the bounds specified
         * by the scalars above (and modifiable with EOCV-Sim's
         * live variable tuner.)
         *
         * Binary meaning that we have either a 0 or 255 value
         * for every pixel.
         *
         * 0 represents our pixels that were outside the bounds
         * 255 represents our pixels that are inside the bounds
         */
        Core.inRange(ycrcbMat, lower, upper, binaryMat);

        /*
         * Release the reusable Mat so that old data doesn't
         * affect the next step in the current processing
         */
        maskedInputMat.release();


        int output = Imgproc.connectedComponentsWithStats(binaryMat, dest_matrix, stats_mat, cens_mat);
        int lineLeftX = 529;

        int lineRightX = 1270;


        double currentmax = 0;
        double bestmax = 0;
        int bestmaxblob = 0;
        for(int i =1; i < output; i++)
        {
            currentmax = stats_mat.get(i, 4)[0];

            if(currentmax > bestmax)
            {
                bestmax = currentmax;
                bestmaxblob = i;
            }
        }

        double x_pos = cens_mat.get(bestmaxblob, 0)[0];
        double y_pos = cens_mat.get(bestmaxblob, 1)[0];


        if(x_pos > 1270)
        {
            result = "RIGHT";
        } else if(x_pos < 529) {
            result = "LEFT";
        }
        else{
            result = "MIDDLE";
        }




        /*
         * Now, with our binary Mat, we perform a "bitwise and"
         * to our input image, meaning that we will perform a mask
         * which will include the pixels from our input Mat which
         * are "255" in our binary Mat (meaning that they're inside
         * the range) and will discard any other pixel outside the
         * range (RGB 0, 0, 0. All discarded pixels will be black)
         */
        Core.bitwise_and(input, input, maskedInputMat, binaryMat);

        /**
         * Add some nice and informative telemetry messages
         */


        telemetry.addData("place", result);
        telemetry.addData("output", output);
        telemetry.addData("stats", stats_mat);
        telemetry.addData("stats", cens_mat);
        telemetry.addData("_________________________________________", "");
        telemetry.addData("bestmax", bestmax);
        telemetry.addData("bestmaxblob",bestmaxblob);
        telemetry.addData("centroid x", cens_mat.get(bestmaxblob, 0)[0]);
        telemetry.addData("centroid y", cens_mat.get(bestmaxblob, 1)[0]);


        telemetry.addData("[>]", "Change these values in tuner menu");
        telemetry.addData("[Color Space]", colorSpace.name());
        telemetry.addData("[Lower Scalar]", lower);
        telemetry.addData("[Upper Scalar]", upper);
        telemetry.update();

        Imgproc.line(maskedInputMat, new Point(lineLeftX, 0), new Point(lineLeftX, input.rows()), new Scalar(0, 255, 0), 20);
        //Imgproc.line(input, new Point(lineMiddleX, 0), new Point(lineMiddleX, input.rows()), new Scalar(0, 255, 0), 20);
        Imgproc.line(maskedInputMat, new Point(lineRightX, 0), new Point(lineRightX, input.rows()), new Scalar(0, 255, 0), 20);

        /*
         * The Mat returned from this method is the
         * one displayed on the viewport.
         *
         * To visualize our threshold, we'll return
         * the "masked input mat" which shows the
         * pixel from the input Mat that were inside
         * the threshold range.
         */
        return  maskedInputMat;
        // return  maskedInputMat;
    }



}