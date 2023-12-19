package org.firstinspires.ftc.teamcode.pipeline;
import org.firstinspires.ftc.teamcode.utility.FieldPosition;
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
    public FieldPosition fieldPosition;
    public Rect leftSpike;
    public Rect centerSpike;
    public Rect rightSpike;

    public int selectedRect = 0;
    Mat hsvMat = new Mat();
    Mat destMat = new Mat();
    Mat detectionMat = new Mat();

    public void setFieldPosition(FieldPosition fieldPosition) {
        this.fieldPosition = fieldPosition;
    }
    @Override
    public Mat processFrame(Mat input) {
        switch(fieldPosition){
            case BLUE_FIELD_LEFT:
            case RED_FIELD_LEFT:
                leftSpike = new Rect(4, 156, 380, 288); //bottom left coords 384, 444
                centerSpike = new Rect(428, 157, 682, 221); //bottom left coords 1110, 397
                break;
            case BLUE_FIELD_RIGHT:
            case RED_FIELD_RIGHT:
                rightSpike = new Rect(4, 156, 380, 288); //bottom left coords 384, 444
                centerSpike = new Rect(428, 157, 682, 221); //bottom left coords 1110, 397
                break;
        }


        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(hsvMat, detectionMat, 1);
        Imgproc.cvtColor(detectionMat, destMat, Imgproc.COLOR_GRAY2RGB);

        drawRectangles(destMat);

        return destMat;
    }

    int findRectangle(Mat input) {
        double leftSpikeSaturation = 0;
        double centerSpikeSaturation = 0;
        double rightSpikeSaturation = 0;

        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

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

        // assume one with the most saturation (least gray) is our TSE
        if ((leftSpikeSaturation > centerSpikeSaturation) && (leftSpikeSaturation > rightSpikeSaturation)) {
            return 1;
            } else if ((centerSpikeSaturation > leftSpikeSaturation) && (centerSpikeSaturation > rightSpikeSaturation)) {
            return 2;
            }
        return 3; // It is either 3 or we can’t tell
        }

        protected double getAvgSaturation(Mat input, Rect rect) {
        Mat submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
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