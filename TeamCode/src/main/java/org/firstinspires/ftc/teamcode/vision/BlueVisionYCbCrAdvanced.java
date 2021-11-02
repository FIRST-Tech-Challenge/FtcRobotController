package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.ObjectDetectionMethods.detect;
import static org.firstinspires.ftc.teamcode.vision.ObjectDetectionMethods.getPositionInformation;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;

public class BlueVisionYCbCrAdvanced extends OpenCvPipeline {

    private volatile boolean[] positions;

    public Scalar x = new Scalar(87,179,232);
    public Scalar low = new Scalar(0, 87.8, 138.8, 0), high =new Scalar(255, 117.6, 194.1, 255);
    public Scalar[] colorRangeTape = {new Scalar(0, 87.8, 138.8, 0), new Scalar(255, 117.6, 194.1, 255)};
    public Scalar[] colorRangeCapstone = {new Scalar(0,0,0,0), new Scalar(0,0,0,255)};


    Telemetry telemetry;
    public BlueVisionYCbCrAdvanced(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Process frame
        Imgproc.cvtColor(input,input,Imgproc.COLOR_RGB2YCrCb);

        // Return information
        ArrayList<MatOfPoint> capstones = detect(input, colorRangeCapstone, new double[]{0,1,0.3,0.7}, new double[]{0.1,0.1});
        MatOfPoint capstone = capstones.get(0);
        ArrayList<MatOfPoint> tape = detect(input, colorRangeTape, new double[]{0,1,0.5,0.7}, new double[]{0.02,0.02});

        int tapeLeft = 0, tapeRight = 0;
        int[] capstonePos = getPositionInformation(capstone);

        for(MatOfPoint m : tape) {
            int[] pos = getPositionInformation(m);
            if(pos[0] - capstonePos[0] > input.width()/20) {
                tapeRight++;
            }
            else if(pos[0] - capstonePos[0] < -1*input.width()/20) {
                tapeLeft++;
            }
        }
        if(tapeRight > tapeLeft) {
            positions = new boolean[]{false,false,true};
        }if(tapeLeft < tapeRight) {
            positions = new boolean[]{true,false,false};
        }else {
            positions = new boolean[]{false,true,false};
        }

        telemetry.addData("validContours",String.valueOf(tape.size()));
        telemetry.addData("position", Arrays.toString(positions));
        telemetry.update();
        Imgproc.drawContours(input, tape,-1, new Scalar(255,0,0), 5);
        Imgproc.drawContours(input, capstones,1, new Scalar(255,0,0), 5);
        return input;
    }

    public boolean[] getPositions() {
        return positions;
    }

}
