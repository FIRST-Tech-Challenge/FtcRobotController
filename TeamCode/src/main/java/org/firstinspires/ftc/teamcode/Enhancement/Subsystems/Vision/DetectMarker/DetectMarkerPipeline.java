package org.firstinspires.ftc.teamcode.Enhancement.Subsystems.Vision.DetectMarker;

import org.firstinspires.ftc.teamcode.Enhancement.Config.MainConfig;
import org.firstinspires.ftc.teamcode.Enhancement.Robot;
import org.firstinspires.ftc.teamcode.Enhancement.Subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;
import org.firstinspires.ftc.teamcode.Util.QuickTelemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * This pipeline detects where the marker is.
 *
 * <p>It does this by splitting the camera input into 3 parts, the Left, Middle, and Right. It
 * checks each part for a custom marker (which is set to be green in the code), or some blue or red
 * tape, dependant on the alliance color.</p>
 *
 * @see org.openftc.easyopencv.OpenCvPipeline
 * @see Vision
 */
public class DetectMarkerPipeline extends OpenCvPipeline {
    private final AllianceColor allianceColor = MainConfig.getAllianceColor();
    private final Rect LEFT_RECT = new Rect(
            new Point(60, 35),
            new Point(110, 75));
    private final Rect MIDDLE_RECT = new Rect(
            new Point(110, 35),
            new Point(150, 75));
    private final Rect RIGHT_RECT = new Rect(
            new Point(150, 35),
            new Point(200, 75));
    private final double PERCENT_COLOR_THRESHOLD = 0.4;
    QuickTelemetry telemetry;
    Mat mask = new Mat();
    private MarkerLocation markerLocation = MarkerLocation.NOT_FOUND;

    /**
     * Class instantiation
     *
     * @param quickTelemetry used for {@link QuickTelemetry}
     * @see Robot
     * @see QuickTelemetry
     * @see AllianceColor
     */
    public DetectMarkerPipeline(QuickTelemetry quickTelemetry) {
        this.telemetry = quickTelemetry;
    }

    /**
     * This method detects where the marker is.
     *
     * <p>It does this by splitting the camera input into left, right, and middle rectangles, these
     * rectangles need to be calibrated. Combined, they do not have to encompass the whole camera
     * input, they probably will only check a small part of it. We then assume the alliance color is
     * either (255, 0, 0) or (0, 0, 255), we get the info when the object is instantiated
     * ({@link #allianceColor}), and that the marker color is (0, 255, 0), which is a bright green
     * ({@link Scalar}'s are used for colors). We compare the marker color with the alliance color
     * on each of the rectangles, if the marker color is on none or multiple of them, it is marked
     * as {@link MarkerLocation#NOT_FOUND}, if otherwise, the respective Location it is in is
     * returned via a {@link MarkerLocation} variable called {@link #markerLocation}</p>
     *
     * @param input A Mask (the class is called {@link Mat})
     * @return The marker location
     * @see #allianceColor
     * @see Mat
     * @see Scalar
     * @see MarkerLocation
     */
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mask, Imgproc.COLOR_RGB2HSV); // TODO: Change COLOR_RGB2HSV to something more useful.
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mask, lowHSV, highHSV, mask);

        Mat left = mask.submat(LEFT_RECT);
        Mat middle = mask.submat(MIDDLE_RECT);
        Mat right = mask.submat(RIGHT_RECT);

        double leftValue = Core.sumElems(left).val[0] / LEFT_RECT.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_RECT.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_RECT.area() / 255;

        left.release();
        middle.release();
        right.release();

        telemetry.telemetry("Left raw value", ((Integer) ((int) Core.sumElems(left).val[0])).toString());
        telemetry.telemetry("Middle raw value", ((Integer) ((int) Core.sumElems(middle).val[0])).toString());
        telemetry.telemetry("Right raw value", ((Integer) ((int) Core.sumElems(right).val[0])).toString());

        telemetry.telemetry("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.telemetry("Middle percentage", Math.round(leftValue * 100) + "%");
        telemetry.telemetry("Right percentage", Math.round(rightValue * 100) + "%");

        boolean markerLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean markerMiddle = middleValue > PERCENT_COLOR_THRESHOLD;
        boolean markerRight = rightValue > PERCENT_COLOR_THRESHOLD;


        if (markerLeft) {
            markerLocation = MarkerLocation.LEFT;
            telemetry.telemetry("Marker Location", "right");
        } else if (markerMiddle) {
            markerLocation = MarkerLocation.MIDDLE;
            telemetry.telemetry("Marker Location", "middle");
        } else if (markerRight) {
            markerLocation = MarkerLocation.RIGHT;
            telemetry.telemetry("Marker Location", "left");
        } else {
            markerLocation = MarkerLocation.NOT_FOUND;
            telemetry.telemetry("Marker Location", "not found");
        }

        Imgproc.cvtColor(mask, mask, Imgproc.COLOR_GRAY2RGB); // TODO: Change COLOR_GRAY2RGB to something more useful.

        Scalar colorNormal;

        if (this.allianceColor == AllianceColor.RED) {
            colorNormal = new Scalar(255, 0, 0); // Pure Red
        } else if (this.allianceColor == AllianceColor.BLUE) {
            colorNormal = new Scalar(0, 0, 255); // Pure Blue
        } else {
            colorNormal = new Scalar(255, 0, 255); // Pure Blue
        }

        Scalar colorMarker = new Scalar(0, 255, 0); // Pure Green

        Imgproc.rectangle(mask, LEFT_RECT, markerLocation == MarkerLocation.LEFT ? colorMarker : colorNormal);
        Imgproc.rectangle(mask, MIDDLE_RECT, markerLocation == MarkerLocation.MIDDLE ? colorMarker : colorNormal);
        Imgproc.rectangle(mask, RIGHT_RECT, markerLocation == MarkerLocation.RIGHT ? colorMarker : colorNormal);

        return mask;
    }

    /**
     * Gets the Marker Location, might be not found because of the Search Status.
     *
     * @return Where the marker is.
     * @see MarkerLocation
     */
    public MarkerLocation getMarkerLocation() {
        return markerLocation;
    }
}
