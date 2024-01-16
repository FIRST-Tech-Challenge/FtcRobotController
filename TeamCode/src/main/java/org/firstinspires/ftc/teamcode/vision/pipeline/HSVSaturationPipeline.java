package org.firstinspires.ftc.teamcode.vision.pipeline;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.vision.util.FieldPosition;
import org.firstinspires.ftc.teamcode.vision.util.SpikePosition;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class HSVSaturationPipeline extends OpenCvPipeline implements VisionProcessor {
    public Scalar nonSelectedColor = new Scalar(0, 255, 0); // Green
    public Scalar selectedColor = new Scalar(0, 0, 255); // Blue

    //1280 x 960 camera resolution

    /* WORK NEEDED TO MAKE A CONSTRUCTOR HERE... If the FieldPosition is RED_FIELD_RIGHT or BLUE_FIELD_RIGHT, this needs to switch.
       Ideally this would be passed from the auto routine (i.e. Auto1_BlueField_Left
     */

    // FieldPosition fieldPosition = FieldPosition.BLUE_FIELD_LEFT; // setting a default to make EOVSim work
    FieldPosition fieldPosition = FieldPosition.BLUE_FIELD_LEFT; // setting a default to make EOVSim work
    Rect leftSpike;
    Rect centerSpike;
    Rect rightSpike;

    SpikePosition spikePos = SpikePosition.UNKNOWN;

    Mat hsvMat = new Mat();
    Mat destMat = new Mat();
    Mat detectionMat = new Mat();

    double leftSpikeSaturation = 0;
    double centerSpikeSaturation = 0;
    double rightSpikeSaturation = 0;

    // These are the values that we have to tune at each competition, different for BLUE_LEFT/RED_LEFT and BLUE_RIGHT/RED_RIGHT.
    static double LEFT_SPIKE_SATURATION_BASELINE = 0;
    static double CENTER_SPIKE_SATURATION_BASELINE = 0;
    static double RIGHT_SPIKE_SATURATION_BASELINE =  0;

    @Override
    public Mat processFrame(Mat input) {
        switch(fieldPosition){
            case BLUE_FIELD_LEFT:
            case RED_FIELD_LEFT:
                LEFT_SPIKE_SATURATION_BASELINE = 28; // this was 22.21125992063492 at calibration
                CENTER_SPIKE_SATURATION_BASELINE = 15; // this was 6.122269404803341 at calibration
                RIGHT_SPIKE_SATURATION_BASELINE =  0;
                leftSpike = new Rect(0, 370, 310, 218); //bottom left coords 384, 444.  Changed from x:0, y:330, w:280, h:288 before calibration.
                centerSpike = new Rect(430, 365, 660, 181); //bottom left coords 1110, 397.  Changed from x:400, y:345, w:650, h:221 before at calibration.
                rightSpike = new Rect(0, 0, 0, 0);
                break;
            case BLUE_FIELD_RIGHT:
            case RED_FIELD_RIGHT:
                LEFT_SPIKE_SATURATION_BASELINE = 0;
                CENTER_SPIKE_SATURATION_BASELINE = 15;
                RIGHT_SPIKE_SATURATION_BASELINE =  28;
                leftSpike = new Rect(0, 0, 0, 0);
                centerSpike = new Rect(250, 365, 660, 181); //bottom left coords 1110, 397.  Change from x:428, y:157, w:682, h:221 before calibration.
                rightSpike = new Rect(960, 370, 310, 221); //bottom left coords 384, 444.  Changed from x:400, y:157, w:180, h:288 before calibration.
                break;
        }


        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(hsvMat, detectionMat, 1);
        Imgproc.cvtColor(detectionMat, destMat, Imgproc.COLOR_GRAY2RGB);

        findRectangle(destMat);
        drawRectangles(destMat);

        return destMat;
    }

    SpikePosition findRectangle(Mat input) {

        switch(fieldPosition) {
            case BLUE_FIELD_LEFT:
            case RED_FIELD_LEFT:
                leftSpikeSaturation = getAvgSaturation(hsvMat, leftSpike);
                centerSpikeSaturation = getAvgSaturation(hsvMat, centerSpike);
                rightSpikeSaturation = getAvgSaturation(hsvMat, rightSpike);

                if (leftSpikeSaturation > LEFT_SPIKE_SATURATION_BASELINE){
                    spikePos = SpikePosition.LEFT;
                } else if (centerSpikeSaturation > CENTER_SPIKE_SATURATION_BASELINE) {
                    spikePos = SpikePosition.CENTRE;
                } else {
                    spikePos = SpikePosition.RIGHT;
                    // It is either 3 or we can’t tell
                }
                break;
            case BLUE_FIELD_RIGHT:
            case RED_FIELD_RIGHT:
                leftSpikeSaturation = getAvgSaturation(hsvMat, leftSpike);
                centerSpikeSaturation = getAvgSaturation(hsvMat, centerSpike);
                rightSpikeSaturation = getAvgSaturation(hsvMat, rightSpike);

                if (rightSpikeSaturation > RIGHT_SPIKE_SATURATION_BASELINE){
                    spikePos = SpikePosition.RIGHT;
                } else if (centerSpikeSaturation > CENTER_SPIKE_SATURATION_BASELINE) {
                    spikePos = SpikePosition.CENTRE;
                } else {
                    spikePos = SpikePosition.LEFT;
                    // It is either 3 or we can’t tell
                }
                break;
        }

        return spikePos;

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

    public SpikePosition getSpikePos() {
        return spikePos;
    }

    public void setFieldPosition(FieldPosition fieldPosition) {
        this.fieldPosition = fieldPosition;
    }

    public void drawRectangles(Mat input) {
        // Only draw relevant field position boxes
        switch(fieldPosition) {
            case BLUE_FIELD_LEFT:
            case RED_FIELD_LEFT:
                Imgproc.rectangle(input, leftSpike, nonSelectedColor);
                Imgproc.rectangle(input, centerSpike, nonSelectedColor);
                break;
            case BLUE_FIELD_RIGHT:
            case RED_FIELD_RIGHT:
                Imgproc.rectangle(input, centerSpike, nonSelectedColor);
                Imgproc.rectangle(input, rightSpike, nonSelectedColor);
                break;
        }

        switch (spikePos) {
            case LEFT:
                Imgproc.rectangle(input, leftSpike, selectedColor);
                break;
            case CENTRE:
                Imgproc.rectangle(input, centerSpike, selectedColor);
                break;
            case RIGHT:
                Imgproc.rectangle(input, rightSpike, selectedColor);
                break;
        }
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        processFrame( frame );
        return(null);
    }
}