package org.firstinspires.ftc.teamcode.vision.dogecv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;


public class LeviColorFilter extends DogeCVColorFilter {
    // Color presets
    public enum ColorPreset{
        RED,
        BLUE,
        YELLOW,
        WHITE
    }

    // Default settings
    private ColorPreset color = ColorPreset.RED;
    private double threshold = -1; // if -1 the color mode will use its own defaults

    // Channels placeholder
    private List<Mat> channels = new ArrayList<>();

    /**
     * Constructor
     * @param filterColor - Color Preset to use (RED,BLUE,YELLOW,WHITE)
     */
    public LeviColorFilter(ColorPreset filterColor){
        updateSettings(filterColor, -1);
    }
    /**
     * Constructor
     * @param filterColor - Color Preset to use (RED,BLUE,YELLOW,WHITE)
     * @param filterThreshold - Threshold value
     */
    public LeviColorFilter(ColorPreset filterColor, double filterThreshold){
        updateSettings(filterColor, filterThreshold);
    }

    private void updateSettings(ColorPreset filterColor, double filterThreshold){
        color = filterColor;
        threshold = filterThreshold;
    }

    /**
     * Process a image and return a mask
     * @param input - Input image to process
     * @param mask - Output mask
     */
    @Override
    public void process(Mat input, Mat mask) {
        channels = new ArrayList<>();

        switch(color){
            case RED:
                if(threshold == -1){
                    threshold = 164;
                }

                Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2Lab);
                Imgproc.GaussianBlur(input,input,new Size(3,3),0);
                Core.split(input, channels);
                Imgproc.threshold(channels.get(1), mask, threshold, 255, Imgproc.THRESH_BINARY);
                break;
            case BLUE:
                if(threshold == -1){
                    threshold = 145;
                }

                Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YUV);
                Imgproc.GaussianBlur(input,input,new Size(3,3),0);
                Core.split(input, channels);
                Imgproc.threshold(channels.get(1), mask, threshold, 255, Imgproc.THRESH_BINARY);
                break;
            case WHITE:
                if(threshold == -1) {
                    threshold = 150;
                }

                Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2Lab);
                Imgproc.GaussianBlur(input,input,new Size(3,3),0);
                Core.split(input, channels);
                Core.inRange(channels.get(0), new Scalar(threshold, 150, 40), new Scalar(255, 150, 150), mask);
                break;
            case YELLOW:
                if(threshold == -1){
                    threshold = 70;
                }

                Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YUV);
                Imgproc.GaussianBlur(input,input,new Size(3,3),0);
                Core.split(input, channels);
                if(channels.size() > 0){
                    Imgproc.threshold(channels.get(1), mask, threshold, 255, Imgproc.THRESH_BINARY_INV);
                }

                break;
        }

        for(int i=0;i<channels.size();i++){
            channels.get(i).release();
        }

        input.release();

    }

    // RED FILTER

}