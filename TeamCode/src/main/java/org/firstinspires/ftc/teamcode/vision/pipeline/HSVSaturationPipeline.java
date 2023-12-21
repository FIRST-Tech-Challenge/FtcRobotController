package org.firstinspires.ftc.teamcode.vision.pipeline;
import org.firstinspires.ftc.teamcode.vision.util.FieldPosition;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class HSVSaturationPipeline extends OpenCvPipeline {
    public Scalar nonSelectedColor = new Scalar(0, 255, 0);
    public Scalar selectedColor = new Scalar(0, 0, 255);



    //1280 x 960 camera resolution
    public FieldPosition fieldPosition = FieldPosition.BLUE_FIELD_LEFT; // setting a default to make EOVSim work
    public Rect leftSpike;
    public Rect centerSpike;
    public Rect rightSpike;

    public int selectedRect = 1;
    Mat hsvMat = new Mat();
    Mat destMat = new Mat();
    Mat detectionMat = new Mat();

    double leftSpikeSaturation = 0;
    double centerSpikeSaturation = 0;
    double rightSpikeSaturation = 0;

    // These are the values that we have to tune at each competition
    static double LEFT_SPIKE_SATURATION_BASELINE = 22.21125992063492;
    static double CENTER_SPIKE_SATURATION_BASELINE =6.122269404803341;
    static double RIGHT_SPIKE_SATURATION_BASELINE =  0;

    public void setFieldPosition(FieldPosition fieldPosition) {
        this.fieldPosition = fieldPosition;
    }

    @Override
    public Mat processFrame(Mat input) {
        switch(fieldPosition){
            case BLUE_FIELD_LEFT:
            case RED_FIELD_LEFT:
                leftSpike = new Rect(0, 330, 280, 288); //bottom left coords 384, 444
                centerSpike = new Rect(400, 345, 650, 221); //bottom left coords 1110, 397
                break;
            case BLUE_FIELD_RIGHT:
            case RED_FIELD_RIGHT:
                rightSpike = new Rect(400, 157, 180, 288); //bottom left coords 384, 444
                centerSpike = new Rect(428, 157, 682, 221); //bottom left coords 1110, 397
                break;
        }


        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(hsvMat, detectionMat, 1);
        Imgproc.cvtColor(detectionMat, destMat, Imgproc.COLOR_GRAY2RGB);

        findRectangle(destMat);
        drawRectangles(destMat);

        return destMat;
    }

    int findRectangle(Mat input) {


        switch(fieldPosition) {
            case BLUE_FIELD_LEFT:
            case RED_FIELD_LEFT:
                leftSpikeSaturation = getAvgSaturation(hsvMat, leftSpike);
                centerSpikeSaturation = getAvgSaturation(hsvMat, centerSpike);
                break;
            case BLUE_FIELD_RIGHT:
            case RED_FIELD_RIGHT:
                centerSpikeSaturation = getAvgSaturation(hsvMat, centerSpike);
                rightSpikeSaturation = getAvgSaturation(hsvMat, rightSpike);
                break;
        }

        System.out.println(leftSpikeSaturation);
        System.out.println(centerSpikeSaturation);
        System.out.println(rightSpikeSaturation);

        if(leftSpikeSaturation > LEFT_SPIKE_SATURATION_BASELINE){
            selectedRect = 1;
            return 1;
        } else if (centerSpikeSaturation > CENTER_SPIKE_SATURATION_BASELINE) {
            selectedRect = 2;
            return 2;

        } else {
            selectedRect = 3;
            return 3;
        }
       // It is either 3 or we can’t tell
    }

        protected double getAvgSaturation(Mat input, Rect rect) {
        Mat submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
        }

    public double getLeftSpikeSaturation() {
        return leftSpikeSaturation;
    }

    public double getCenterSpikeSaturation() {
        return centerSpikeSaturation;
    }

    public double getRightSpikeSaturation() {
        return rightSpikeSaturation;
    }


    public void drawRectangles(Mat input) {
        Imgproc.rectangle(input, leftSpike, nonSelectedColor);
        Imgproc.rectangle(input, centerSpike, nonSelectedColor);

        switch (selectedRect) {
            case 1:
                Imgproc.rectangle(input, leftSpike, selectedColor);
                break;
            case 2:

                Imgproc.rectangle(input, centerSpike, selectedColor);
                break;
        }
    }

}