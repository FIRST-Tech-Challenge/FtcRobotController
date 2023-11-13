package org.firstinspires.ftc.team15091;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class RBPipeline extends PipelineBase {
    Mat temp = new Mat(), sub;
    String debug = "";
    Double[] hues = new Double[2];
    PixelPosition position = PixelPosition.Right;

    // Define region for left and middle
    static final Rect maskLeft = new Rect(0, 190, 60, 50);
    static final Rect maskMiddle = new Rect(320, 165, 60, 50);

    @Override
    public Mat processFrame(Mat input) {
        // Convert to RBG to HSV, so we can extract HUE channel
        Imgproc.cvtColor(input, temp, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(temp, temp, Imgproc.COLOR_RGB2HSV);
//        Imgproc.GaussianBlur(temp, temp, new Size(5, 5), 0);

        sub = temp.submat(maskLeft);
        hues[0] = Core.mean(sub).val[0];
        sub.release();

        sub = temp.submat(maskMiddle);
        hues[1] = Core.mean(sub).val[0];
        sub.release();
        temp.release();

        // region Visual Aid
        /*
         * Draw a rectangle showing sample region 1,2 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                maskLeft,
                YELLOW, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines
        Imgproc.rectangle(
                input, // Buffer to draw on
                maskMiddle,
                YELLOW, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines
                // endregion

        if (isRed(hues[0]) || isBlue(hues[0])) {
            position = PixelPosition.Left;
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    maskLeft, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        } else if (isRed(hues[1]) || isBlue(hues[1])) {
            position = PixelPosition.Middle;
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    maskMiddle, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        } else {
            position = PixelPosition.Right;
        }
        debug = String.format(" %3.0f, %3.0f", hues[0], hues[1]);

        return input;
    }

    boolean isRed(double hueValue) {
        return hueValue > 0 && hueValue < 20;
    }

    boolean isBlue(double hueValue) {
        return hueValue > 99 && hueValue < 120;
    }
}