package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.VISION_DATA.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class TSEDetectorPipeline extends OpenCvPipeline {

    private Telemetry telemetry;

    //YCrCb
    private Scalar borderColor = new Scalar(255, 255, 255);
    private Scalar lowerRange = new Scalar(0, 0, 0);
    private Scalar upperRange = new Scalar(255, 255, 255);

    private Mat output;
    private Mat YCrCbMat;

    private TSE_POSITION detectedPosition;

    public TSEDetectorPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(output);
        input.copyTo(YCrCbMat);
        try {
            // Convert the frame to YCrCb (this almost completely eliminates light-related error)
            Imgproc.cvtColor(input, YCrCbMat, Imgproc.COLOR_RGB2YCrCb);

            // Draw the margin (GUI)
            Imgproc.rectangle(output, new Rect(LEFT_MARGIN, TOP_MARGIN, CAMERA_WIDTH - LEFT_MARGIN - RIGHT_MARGIN, CAMERA_HEIGHT - TOP_MARGIN - BOTTOM_MARGIN - 2), MARGIN_BOX_COLOR, 2);

            // Draw rectangles around each barcode
            Rect[] rects = new Rect[]{createRectInMargin(LEFT_BOX_X1, LEFT_BOX_Y1, LEFT_BOX_X2, LEFT_BOX_Y2),
                                      createRectInMargin(MIDDLE_BOX_X1, MIDDLE_BOX_Y1, MIDDLE_BOX_X2, MIDDLE_BOX_Y2),
                                      createRectInMargin(RIGHT_BOX_X1, RIGHT_BOX_Y1, RIGHT_BOX_X2, RIGHT_BOX_Y2)};

            Mat[] subMats = new Mat[3];        // This stores the sub-mats for each region
            // This stores the Cr and Cb mat for each section
            Mat[][] channels = new Mat[][]{{new Mat(), new Mat()}, {new Mat(), new Mat()}, {new Mat(), new Mat()}};
            Scalar[][] means = new Scalar[3][2];    // This stores the average Chroma for each section

            // Process each box
            for (int i = 0; i < 3; i++) {
                // Draw a rectangle (GUI)
                Imgproc.rectangle(output, rects[i], BOX_COLORS[i], 1);

                // Extract a sub-mat, chroma keys, and chroma averages
                subMats[i] = YCrCbMat.submat(rects[i]);

                // Repeat for both the red and blue chroma keys
                for (int j = 0; j < 2; j++) {
                    Core.extractChannel(subMats[i], channels[i][j], j+1); // Extract the Chroma
                    means[i][j] = Core.mean(channels[i][j]);
                }
            }
            // Display some telemetry data
            telemetry.addData("Left-Cr", Math.rint(means[0][0].val[0]));
            telemetry.addData("Left-Cb", Math.rint(means[0][1].val[0]));
            telemetry.addData("Middle-Cr", Math.rint(means[1][0].val[0]));
            telemetry.addData("Middle-Cb", Math.rint(means[1][1].val[0]));
            telemetry.addData("Right-Cr", Math.rint(means[2][0].val[0]));
            telemetry.addData("Right-Cb", Math.rint(means[2][1].val[0]));

            telemetry.addData("complete", "yay!");
            telemetry.update();

            detectedPosition = TSE_POSITION.UNKNOWN;
            if (means[0][1].val[0] < 102) { detectedPosition = TSE_POSITION.LEFT; }
            if (means[1][1].val[0] < 102) { detectedPosition = TSE_POSITION.MIDDLE; }
            if (means[2][1].val[0] < 102) { detectedPosition = TSE_POSITION.RIGHT; }

            Imgproc.putText(output, detectedPosition.asString, new Point(25, CAMERA_HEIGHT - 25), 0, 1, RGB_WHITE, 1);
        } catch (Exception e){
            telemetry.addData("Error", "THERE WAS AN ERROR");
            telemetry.update();
        }

        return output;
    }

    private Rect createRectInMargin(int x1, int y1, int x2, int y2) {
        return new Rect(x1 + LEFT_MARGIN, y1 + TOP_MARGIN, Math.abs(x2 - x1), Math.abs(y2 - y1));
    }

    public TSE_POSITION getPos() {
        return this.detectedPosition;
    }

}
