package org.firstinspires.ftc.teamcode.vision.dogecv;

import org.opencv.core.Mat;

public abstract class DogeCVColorFilter {
    public abstract void process(Mat input, Mat mask);
}