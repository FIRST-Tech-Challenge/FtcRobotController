package org.firstinspires.ftc.teamcode.vision.dogecv;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;

/**
 * Created by Victo on 9/10/2018.
 */

public class RatioScorer {

    public double weight       = 1.0;
    public double perfectRatio = 1.0;
    public RatioScorer(){

    }

    /**
     * Constructor
     * @param perfectRatio - Perfect ratio of height / width (abs value)
     * @param weight - How much to weight the final score (1-10 is usually good)
     */
    public RatioScorer(double perfectRatio, double weight){
        this.weight = weight;
        this.perfectRatio = perfectRatio;
    }
    /**
     * @param input - Input mat (Can be MatOfPoint for contours)
     * @return - Difference from perfect score
     */
    public double calculateScore(Mat input) {
        if(!(input instanceof MatOfPoint)) return Double.MAX_VALUE;
        MatOfPoint contour = (MatOfPoint) input;
        double score = Double.MAX_VALUE;

        // Get bounding rect of contour
        Rect rect = Imgproc.boundingRect(contour);
        double x = rect.x;
        double y = rect.y;
        double w = rect.width;
        double h = rect.height;

        double cubeRatio = Math.max(Math.abs(h/w), Math.abs(w/h)); // Get the ratio. We use max in case h and w get swapped??? it happens when u account for rotation
        double ratioDiffrence = Math.abs(cubeRatio - perfectRatio);
        return ratioDiffrence * weight;
    }
}
