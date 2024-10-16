package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;


import java.util.concurrent.atomic.AtomicReference;

/*
1) you can remove CameraStreamSource
2) implements just means that they made like an interface called "VisionProcessor" that tells
you what functions you NEED to have, but it doesn't have it's own version of them. So that's why
you have to write the init() and processFrame() methods here
 */
@Config
public class NewBlueRightProcessor implements VisionProcessor, CameraStreamSource {

    // you can delete this
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    /*
    So OpenCv represents any image as a Mat (Matrix, it took me so long to make this connection).
    The top left corner is (0, 0), and the bottom right corner is (width, height).
    These Mats will all be holding data later, but we don't want to create the objects over and over,
    so we declare and instantiate them all here, then just re-assign them later. This is important
    and if it is instead declared inside "processFrame" can crash the robot because it ran out of
    memory.
     */
    private Mat testMat = new Mat();
    private Mat highMat = new Mat();
    private Mat lowMat = new Mat();
    private Mat finalMat = new Mat();

    public static double rightNW_X = 0.65;
    public static double rightNW_Y = 0.28;
    public static double rightSE_X = 0.89;
    public static double rightSE_Y = 0.62;

    private double middleThreshold = 0.3;
    private double rightThreshold = 0.3;
    Telemetry telemetry;

    PropPositions propLocation;
    double middlePerc;
    double rightPerc;

    Rect MIDDLE_RECTANGLE;
    Rect RIGHT_RECTANGLE;
    private boolean detectingRed = true;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

		/*
		So here we're creating the rectangle. By themselves they don't do anything, but we're going to
		use them later to define a rectangular region of interest (where we want to try detecting).
		As mentioned earlier, the top left corner is (0, 0) and the bottom right corner is (width, height)
		Here the two points are the top left and bottom right of the regions of interest (ROIs).
		So if you want to cut out the top 1/2, the y coordinate of your first point should be 0.5 * height.
		I had the multipliers as fractions before, but I'm changing them to the equivalent decimals.
		I think it's easier to move them around in decimals.
		 */

        this.MIDDLE_RECTANGLE = new Rect(
                new Point(0.175 * width, 0.15 * height),
                new Point(0.45 * width, 0.4 * height)
        );

        this.RIGHT_RECTANGLE = new Rect(
                new Point(rightNW_X * width, rightNW_Y * height),
                new Point(rightSE_X * width, rightSE_Y * height)
        );

        // you can delete this
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    /*
    I think this one is pretty self explanatory. You have to call this function BEFORE you add
    it to the VisionPortal. Also, it defaults to detecting red
     */
    public void setDetectionColor(boolean detectRed) {
        this.detectingRed = detectRed;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
		/*
		So here, we're taking the input "frame", which an RGB image and converting it to HSV, then
		storing the HSV output in "testMat". We use HSV (Hue, Saturation, Value) because it's easier
		to separate out the colors you want vs. don't want. Especially because the "lightness" (value)
		of the color is it's own channel. RGB is additive, so if you maintain the same proportion of
		each R, G, and B, but maybe double the values, it's the same color, but more saturated, and
		brighter. This makes it difficult to only detect what you want.
		 */
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);

        if(this.detectingRed) {

			/*
			You can trust that these values probably work for you too. But if you are interested in
			doing this process yourself, look up "HSV Cylinder" that shows how HSV can be visualized.
			Red-ish colors are both at the start of the circle and end of the circle so we have to
			detect both parts then look at all detected regions.

			NOTE: "real" HSV uses a 0-360 system for Hue. The others are sometimes 0-1, 0-255, or 0-360.
			OpenCv uses the ranges (0-179, 0-255, 0-255) so you should halve whatever Hue value you
			find online.
			 */
            Scalar lowHSVRedLower = new Scalar(0, 100, 20);  //Beginning of Color Wheel
            Scalar lowHSVRedUpper = new Scalar(10, 255, 255);

            Scalar redHSVRedLower = new Scalar(160, 100, 20); //Wraps around Color Wheel
            Scalar highHSVRedUpper = new Scalar(179, 255, 255);

			/*
			Core.inRange takes "testMat" as an input, checks every pixel to see if it's within
			"lowHSVRedLower" and "lowHSVRedUpper". If it is, it makes the corresponding pixel on
			"lowMat" white. If it isn't, it makes it black. So now "lowMat" is a binary image.
			 */
            Core.inRange(testMat, lowHSVRedLower, lowHSVRedUpper, lowMat);
            Core.inRange(testMat, redHSVRedLower, highHSVRedUpper, highMat);

            // bitwise_or just says, if it's white in "highMat" or "lowMat" add it to "finalMat"
            Core.bitwise_or(lowMat, highMat, finalMat);
        } else {
            Scalar blueHSVLower = new Scalar(85, 100, 20);
            Scalar blueHSVUpper = new Scalar(140, 255, 255);

            Core.inRange(testMat, blueHSVLower, blueHSVUpper, finalMat);
        }

        // don't release. This is leftover from before VisionProcessor. It's bad now.
//		testMat.release();
//
//		lowMat.release();
//		highMat.release();

		/*
		So a "submat" is just looking at a specific portion of a Mat. In this case, we look at
		the part of "finalMat" defined by the coordinates of LEFT_RECTANGLE. Then we just add up
		all the values.
		 */
        double rightBox = Core.sumElems(finalMat.submat(RIGHT_RECTANGLE)).val[0];
        double middleBox = Core.sumElems(finalMat.submat(MIDDLE_RECTANGLE)).val[0];

		/*
		Here we're getting the percentage of the submat that is white. First, we divide by the area
		because a bigger area will just naturally have more detected stuff in it. Then we divide by 255
		because white is represented as 255 and black as 0. This makes it an actual percent
		 */
        this.rightPerc = rightBox / RIGHT_RECTANGLE.area() / 255;
        this.middlePerc = middleBox / MIDDLE_RECTANGLE.area() / 255; //Makes value [0,1]

		/*
		I won't comment this out right now, but it really should be. This was from before I
		added middleThreshold and leftThreshold and just tried to compare both to the same threshold.
		 */
        this.middlePerc *= 2;

        // this part should make sense
        if(rightPerc > middlePerc && rightPerc > rightThreshold) {
            propLocation = PropPositions.RIGHT;
        } else if (middlePerc > rightPerc && middlePerc > middleThreshold) {
            propLocation = PropPositions.MIDDLE;
        } else {
            propLocation = PropPositions.LEFT;
        }

		/*This line should only be added in when you want to see your custom pipeline
		on the driver station stream, do not use this permanently in your code as
		you use the "frame" mat for all of your pipelines, such as April Tags*/
//		finalMat.copyTo(frame);
        Scalar redBorder = new Scalar(255, 0, 0);
        Scalar greenBorder = new Scalar(0, 255, 0);

        // just draws a red border if it doesn't see it and a green border if it does.
        switch (propLocation) {
            case LEFT:
                Imgproc.rectangle(frame, RIGHT_RECTANGLE, redBorder);
                Imgproc.rectangle(frame, MIDDLE_RECTANGLE, redBorder);
                break;
            case MIDDLE:
                Imgproc.rectangle(frame, MIDDLE_RECTANGLE, greenBorder);
                Imgproc.rectangle(frame, RIGHT_RECTANGLE, redBorder);
                break;
            case RIGHT:
                Imgproc.rectangle(frame, RIGHT_RECTANGLE, greenBorder);
                Imgproc.rectangle(frame, MIDDLE_RECTANGLE, redBorder);
                break;
        }

        // you can delete these 3 lines.
        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);

        return null;

//		return null; //You do not return the original mat anymore, instead return null

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }

    public PropPositions getPropLocation() {
        return this.propLocation;
    }

    public double[] getPercents() {
        return new double[]{rightPerc, middlePerc};
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void updateTelemetry() {
        telemetry.addLine("Prop Processor")
                .addData("Left Percent", rightPerc)
                .addData("Middle Percent", middlePerc)
                .addData("Prop Location", propLocation);
    }

    // you can delete this function
    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    public enum PropPositions {
        LEFT,
        MIDDLE,
        RIGHT,
        UNFOUND;
    }
}