package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class JunctionDetectPipeline extends ColorDetectPipeline {
    @Override
    public Mat processFrame(Mat input) {
        if(input == null) { throw new IllegalArgumentException("Input cannot be null"); }
        int[] ca = {43, 255, 255};
        int[] co = {50, 25, 200};
        return processFrameWithRange(input, ca, co);
    }
}
