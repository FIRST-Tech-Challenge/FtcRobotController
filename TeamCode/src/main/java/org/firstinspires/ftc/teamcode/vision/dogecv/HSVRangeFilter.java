package org.firstinspires.ftc.teamcode.vision.dogecv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class HSVRangeFilter extends DogeCVColorFilter{

    private Scalar lower = new Scalar(255,255,255); // Lower HSV Color
    private Scalar upper = new Scalar(0,0,0); // Upper HSV Color

    /**
     * Constructor
     * @param lower - Lower Color
     * @param upper - Upper Color
     */
    public HSVRangeFilter(Scalar lower, Scalar upper){
        updateSettings(lower, upper);
    }

    /**
     * Update the filter settings
     * @param lower - Lower Color
     * @param upper - Upper Color
     */
    public void updateSettings(Scalar lower, Scalar upper){
        this.lower = lower;
        this.upper = upper;
    }
    /**
     * Process a image and return a mask
     * @param input - Input image to process
     * @param mask - Output mask
     */
    @Override
    public void process(Mat input, Mat mask) {
        // Convert the input to HSV
        Imgproc.cvtColor(input,input,Imgproc.COLOR_RGB2HSV_FULL);

        // Blur it
        Imgproc.GaussianBlur(input,input,new Size(5,5),0);
        // Run in range check
        Core.inRange(input,lower,upper,mask);
        input.release();
    }
}
