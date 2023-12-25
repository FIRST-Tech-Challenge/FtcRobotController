package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;

public class SpikeDetectorPipeline extends OpenCvPipeline {

    Rect z1Rect = new Rect(20, 280, 150, 150);
    Rect z2Rect = new Rect(220, 240, 150, 150);
    Rect z3Rect = new Rect(420, 280, 150, 150);

    HashMap<Integer, Rect> zoneRects = new HashMap<Integer, Rect>() {{
        put(1, z1Rect);
        put(2, z2Rect);
        put(3, z3Rect);
    }};

    Mat original;

    Scalar grey = new Scalar(93, 93, 93);
    Scalar red = new Scalar(255, 0, 0);
    Scalar blue = new Scalar(0, 0, 255);

    Scalar spikeColor;
    int spikeZone = -1;

    public SpikeDetectorPipeline(Scalar spikeColor) {
        super();
        this.spikeColor = spikeColor;
    }


    @Override
    public Mat processFrame(Mat frame) {
        double blueDist[] = new double[3];
        double redDist[] = new double[3];
        Scalar z1AvgColor, z2AvgColor, z3AvgColor;
        Mat z1, z2, z3;

        //Creating duplicate of original frame with no edits
        original = frame.clone();

        z1 = frame.submat(z1Rect);
        z2 = frame.submat(z2Rect);
        z3 = frame.submat(z3Rect);

        z1AvgColor = Core.mean(z1);
        z2AvgColor = Core.mean(z2);
        z3AvgColor = Core.mean(z3);


        blueDist[0] = colorDist(z1AvgColor, blue);
        blueDist[1] = colorDist(z2AvgColor, blue);
        blueDist[2] = colorDist(z3AvgColor, blue);

        redDist[0] = colorDist(z1AvgColor, red);
        redDist[1] = colorDist(z2AvgColor, red);
        redDist[2] = colorDist(z3AvgColor, red);

        int bluemin = 0;
        int redmin = 0;
        for (int i = 1; i < 3; i++) {
            if (blueDist[i] < blueDist[bluemin]) {
                bluemin = i;
            }
            if (redDist[i] < redDist[redmin]) {
                redmin = i;
            }
        }

        spikeColor = blue;
        spikeZone = bluemin + 1;
        if (redDist[redmin] < blueDist[bluemin]) {
            spikeZone = redmin + 1;
            spikeColor = red;
        }

        for (int z = 1; z <= 3; z++) {
            Scalar color = grey;
            if (z == spikeZone)
                color = spikeColor;
            Imgproc.rectangle(
                    frame,
                    Objects.requireNonNull(zoneRects.get(z)),
                    color,
                    4
            );
        }
        return frame;
    }

    public double colorDist(Scalar c1, Scalar c2){
        double rDiff = c1.val[0] - c2.val[0];
        double gDiff = c1.val[1] - c2.val[1];
        double bDiff = c1.val[2] - c2.val[2];
        return Math.sqrt(rDiff * rDiff + gDiff * gDiff + bDiff * bDiff);
    }


    public int getSpikeZone() {
        return spikeZone;
    }
    public Scalar getSpikeColor() {
        return spikeColor;
    }
}
