package org.firstinspires.ftc.teamcode.Configs.newConfig;


import static org.firstinspires.ftc.teamcode.Configs.newConfig.Direction.NOT_INITIALIZED;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.android.dx.rop.cst.CstArray;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class BasePipe extends OpenCvPipeline {
    private Telemetry telemetry;

    public BasePipe(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat hsvIMG = new Mat();
        List<Mat> channels = new ArrayList<>();

        Imgproc.cvtColor(input, hsvIMG, Imgproc.COLOR_BGR2HSV);
        Core.split(hsvIMG, channels);

        Mat red = new Mat();
        Mat blue = new Mat();
        Mat green = new Mat();
        Mat yellow = new Mat();
        Mat orange = new Mat();

        Core.inRange(channels.get(0), new Scalar(0), new Scalar(25), red);
        Core.inRange(channels.get(0), new Scalar(100), new Scalar(115), blue);
        Core.inRange(channels.get(0), new Scalar(55), new Scalar(65), green);
        Core.inRange(channels.get(0), new Scalar(40), new Scalar(50), yellow);
        Core.inRange(channels.get(0), new Scalar(30), new Scalar(40), orange);
        //blue values seem to be more inclined to orange objects

        double imgSize = hsvIMG.cols() * hsvIMG.rows();
        double redPercent = ((double) Core.countNonZero(red))/imgSize * 100;
        double bluePercent = ((double) Core.countNonZero(blue))/imgSize * 100;
        double greenPercent = ((double) Core.countNonZero(green))/imgSize * 100;
        double yellowPercent = ((double) Core.countNonZero(yellow))/imgSize * 100;
        double orangePercent = ((double) Core.countNonZero(orange))/imgSize * 100;
        red.release();
        blue.release();
        green.release();
        yellow.release();
        green.release();
        orange.release();
        hsvIMG.release();
        telemetry.addData("Color Value Percents", "\n" +
                 "Red "+ Math.ceil(redPercent) + "\n" +
                 "blue " + Math.ceil(bluePercent) + "\n" +
                 "green " + Math.ceil(greenPercent) + "\n" +
                 "yellow " + Math.ceil(yellowPercent) + "\n" +
                 "orange " + Math.ceil(orangePercent) + "\n"
        );
        telemetry.update();

        return input;
    }
}
