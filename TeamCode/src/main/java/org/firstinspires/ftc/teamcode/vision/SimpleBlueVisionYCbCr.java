package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.VisionThing.VisionThing;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class SimpleBlueVisionYCbCr extends OpenCvPipeline {
    Telemetry telemetry = null;

    public SimpleBlueVisionYCbCr(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    public SimpleBlueVisionYCbCr() {}

    public Scalar low = new Scalar(0, 83.6, 79.3, 0), high = new Scalar(255, 110.5, 128.9, 255);

    @Override
    public Mat processFrame(Mat input) {
        // YCbCr scalars
        ArrayList<VisionThing> tape = DetectionMethods.detectYCrCb(input, new Scalar(0, 83.6, 79.3, 0), new Scalar(255, 110.5, 177.1, 255), 0,
                1,0.4,1,0.05,0.1,"tape");
        ArrayList<VisionThing> capstone = DetectionMethods.detectYCrCb(input, low, high, 0,
                1,0,1,0.1,0.2,"capstone");

        for(VisionThing v : tape) {
            Imgproc.circle(input,new Point(v.x,v.y), (int) v.magSize(),new Scalar(255,0,0,0),5);
            if(telemetry != null) {
                telemetry.addLine(v.toString());
            }
        }
        for(VisionThing v: capstone) {
            Imgproc.circle(input,new Point(v.x,v.y), (int) v.magSize(),new Scalar(0,255,0,0),5);
            if(telemetry != null) {
                telemetry.addLine(v.toString());
            }
        }
        if(telemetry != null) {
            telemetry.update();
        }
        return input;
    }
}
