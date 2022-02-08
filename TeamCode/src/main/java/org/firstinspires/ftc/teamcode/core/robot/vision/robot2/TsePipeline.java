package org.firstinspires.ftc.teamcode.core.robot.vision.robot2;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco.Aruco;
import org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco.Dictionary;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/*
red
bottom height = 0.2
bottom width = 0.805
middle height = 0.3
middle width = 0.534
top height = 0.45
top width = 0.195

blue
bottom height = 0.41
bottom width = 0.75
middle height = 0.315
middle width = 0.37
top height 0.25
top width = 0.08
 */
@Config
public class TsePipeline extends OpenCvPipeline {
    private final Mat markerImage = new Mat();

    public void startPipeline() {

    }
    public void stopPipeline() {

    }
    /**
     * @param input input frame matrix
     */
    @Override
    public Mat processFrame(Mat input) {
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_4X4_50);
        Aruco.drawMarker(dictionary, 32, 200, markerImage,1);
        return input;
    }

    public void saveImage() {
        Imgcodecs.imwrite("tse.png", markerImage);
    }
}
