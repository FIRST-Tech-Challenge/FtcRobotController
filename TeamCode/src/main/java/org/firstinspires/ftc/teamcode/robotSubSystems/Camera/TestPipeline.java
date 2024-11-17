package org.firstinspires.ftc.teamcode.robotSubSystems.Camera;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TestPipeline extends OpenCvPipeline {
    //Creates a static Mat variable
    private static Mat material;
    private static Point center;

    @Override
    public Mat processFrame(Mat input) {
        input = new Mat(input, Constants.cropRect);
        Imgproc.cvtColor(input, input, Constants.binary);
        Imgproc.blur(input, input, Constants.BlurRadius);
        center = Contours.getCenter(
                Contours.contourPolyList(
                        Contours.getBiggestContour(
                                Contours.getContour(input, Constants.redLowHSV, Constants.redHighHSV)
                        )
                )
                , input);
        if(center == null) {
            return input;
        }
        return input;
    }
    //Cloning the material (input)
    public static Mat getClonedMat() {return material.clone();}

    //Creates a public function so we can use the material (input) in every place needed
    public static Mat getMat() {return material;}
}
