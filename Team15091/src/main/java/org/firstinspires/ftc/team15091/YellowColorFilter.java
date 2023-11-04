package org.firstinspires.ftc.team15091;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class YellowColorFilter {
    // Channels placeholder
    private List<Mat> channels = new ArrayList<>();
    private double _threshold = -1d; // if -1 the color mode will use its own defaults

    public YellowColorFilter() { }

    public void set_threshold(double threshold) {
        _threshold = threshold;
    }

    /**
     * Process a image and return a mask
     * @param input - Input image to process
     * @param mask - Output mask
     */
    public void process(Mat input, Mat mask) {
        channels = new ArrayList<>();

        if(_threshold == -1d){
            _threshold = 75d;
        }

//        Mat lab = new Mat(input.size(), 0);
//        Imgproc.cvtColor(input, lab, Imgproc.COLOR_RGB2Lab);
//        Mat temp = new Mat();
//        Core.inRange(input, new Scalar(0,0,0), new Scalar(255,255,164), temp);
//        Mat mask2 = new Mat(input.size(), 0);
//        temp.copyTo(mask2);
//        input.copyTo(input, mask2);
//        mask2.release();
//        temp.release();
//        lab.release();

        //Convert to RBG to YUV, so we can extract yellow channel
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YUV);
        Imgproc.GaussianBlur(input,input,new Size(3,3),0);
        Core.split(input, channels);
        if(channels.size() > 0){
            Imgproc.threshold(channels.get(1), mask, _threshold, 255, Imgproc.THRESH_BINARY_INV);
        }

        for(int i=0;i<channels.size();i++){
            channels.get(i).release();
        }

        input.release();
    }
}
