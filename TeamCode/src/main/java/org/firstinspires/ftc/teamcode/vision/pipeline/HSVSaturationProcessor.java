package org.firstinspires.ftc.teamcode.vision.pipeline;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.vision.util.FieldPosition;
import org.firstinspires.ftc.teamcode.vision.util.SpikePosition;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class HSVSaturationProcessor implements VisionProcessor {
    public Scalar nonSelectedColor = new Scalar(0, 255, 0); // Green
    public Scalar selectedColor = new Scalar(0, 0, 255); // Blue

    FieldPosition fieldPosition = FieldPosition.NOT_ON_FIELD; // setting a default to make EOVSim work

    Rect leftSpike;
    Rect centerSpike;
    Rect rightSpike;

    SpikePosition spikePos = SpikePosition.UNKNOWN;

    Mat hsvMat = new Mat();
    Mat processedMat = new Mat();
    Mat detectionMat = new Mat();

    double leftSpikeSaturation = 0;
    double centerSpikeSaturation = 0;
    double rightSpikeSaturation = 0;

    // These are the values that we have to tune at each competition, different for BLUE_LEFT/RED_LEFT and BLUE_RIGHT/RED_RIGHT.
    static double LEFT_SPIKE_SATURATION_BASELINE = 0;
    static double CENTER_SPIKE_SATURATION_BASELINE = 0;
    static double RIGHT_SPIKE_SATURATION_BASELINE =  0;


    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        switch(getFieldPosition()){
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


        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(frame, frame, 1);
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_GRAY2RGB);

        findRectangle(frame);
        drawRectangles(frame);

        return null;
    }


    SpikePosition findRectangle(Mat input) {

        switch(getFieldPosition()) {
            case BLUE_FIELD_LEFT:
            case RED_FIELD_LEFT:
                leftSpikeSaturation = getAvgSaturation(input, leftSpike);
                centerSpikeSaturation = getAvgSaturation(input, centerSpike);
                rightSpikeSaturation = getAvgSaturation(input, rightSpike);

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
                leftSpikeSaturation = getAvgSaturation(input, leftSpike);
                centerSpikeSaturation = getAvgSaturation(input, centerSpike);
                rightSpikeSaturation = getAvgSaturation(input, rightSpike);

                if (rightSpikeSaturation > RIGHT_SPIKE_SATURATION_BASELINE){
                    spikePos = SpikePosition.RIGHT;
                } else if (centerSpikeSaturation > CENTER_SPIKE_SATURATION_BASELINE) {
                    spikePos = SpikePosition.CENTRE;
                } else {
                    spikePos = SpikePosition.LEFT;
                    // It is either 3 or we can’t tell
                }
                break;
            case NOT_ON_FIELD:
            default:
                spikePos = SpikePosition.UNKNOWN;
                Rect textRect = new Rect(150, 165, 660, 181);
                msgOnImage( input,"findRectangles: Bot not on field or no field pos given",textRect);
        }

        return spikePos;

    }

    private static void msgOnImage(Mat input, String msg,Rect msgBoxRect) {
        // Define the font and other text properties
        int fontFace = Imgproc.FONT_HERSHEY_SIMPLEX;
        double fontScale = 1.0;
        Scalar fontColor = new Scalar(255, 255, 255); // White color
        int thickness = 2;


        // Get the size of the text
        Size textSize = Imgproc.getTextSize(msg, fontFace, fontScale, thickness, new int[]{0});

        // Calculate the position to center the text in the given rectangle
        Point textPosition = new Point(
                msgBoxRect.x + (msgBoxRect.width - textSize.width) / 2,
                msgBoxRect.y + (msgBoxRect.height + textSize.height) / 2
        );

        // Draw the text on the image
        Imgproc.putText(input,msg , textPosition, fontFace, fontScale, fontColor, thickness);
    }


    public void drawRectangles(Mat input) {
        // Only draw relevant field position boxes
        switch(getFieldPosition()) {
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
            case NOT_ON_FIELD:
            default:
                msgOnImage(input,"drawRectangles: Unknown Field Position or not on field", new Rect(150, 265, 660, 181));
        }

        switch (getSpikePos()) {
            case LEFT:
                Imgproc.rectangle(input, leftSpike, selectedColor);
                break;
            case CENTRE:
                Imgproc.rectangle(input, centerSpike, selectedColor);
                break;
            case RIGHT:
                Imgproc.rectangle(input, rightSpike, selectedColor);
                break;
            case UNKNOWN:
            default:
                msgOnImage(input,"drawRectangles: Unknown Spike Position", new Rect(150, 365, 660, 181));
        }
    }


    /**
     * @param canvas
     * @param onscreenWidth
     * @param onscreenHeight
     * @param scaleBmpPxToCanvasPx
     * @param scaleCanvasDensity
     * @param userContext
     */
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
           // just showing how you can use this onDrawFrame to edit the image going to the DriverStation here rather than directly on the processed image
//            Rect theRect = new Rect(150, 265, 660, 181);
//            Paint paint = new Paint();
//            paint.setColor(Color.GREEN);
//            canvas.drawRect(makeGraphicsRect(theRect,scaleBmpPxToCanvasPx),paint);
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
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

    public FieldPosition getFieldPosition(){
        return fieldPosition;
    }


    public Rect getLeftSpikeBox() {
        return leftSpike;
    }

    public Rect getCenterSpikeBox() {
        return centerSpike;
    }

    public Rect getRightSpikeBox() {
        return rightSpike;
    }

}