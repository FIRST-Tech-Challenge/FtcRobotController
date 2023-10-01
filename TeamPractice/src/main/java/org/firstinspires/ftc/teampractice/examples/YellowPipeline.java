package org.firstinspires.ftc.teampractice.examples;

import org.firstinspires.ftc.teampractice.PipelineBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class YellowPipeline extends PipelineBase {
    final Rect mask1 = new Rect(5, 60, 50, 30);
    final Rect mask2 = new Rect(160, 60, 50, 30);

    private List<Mat> channels = new ArrayList<>();
    Mat hierarchy = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        frameTemp = new Mat();

        //Convert to RBG to YUV, so we can extract yellow channel
        Imgproc.cvtColor(input, frameTemp, Imgproc.COLOR_RGB2YUV);
        Imgproc.GaussianBlur(frameTemp, frameTemp, new Size(3, 3), 0);

        Core.split(frameTemp, channels);
        if (channels.size() > 0) {
            Imgproc.threshold(channels.get(1), frameTemp, 80d, 255, Imgproc.THRESH_BINARY_INV);
        }

        for (int i = 0; i < channels.size(); i++) {
            channels.get(i).release();
        }

        Mat region1 = frameTemp.submat(mask1);
        Mat region2 = frameTemp.submat(mask2);

        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */
        double avg1 = Core.mean(region1).val[0];
        double avg2 = Core.mean(region2).val[0];

        if (avg1 > 0d) // Was it from region 1?
        {
            Imgproc.rectangle(input, mask1, YELLOW, -1);
        } else if (avg2 > 0d) // Was it from region 2?
        {
            Imgproc.rectangle(input, mask2, YELLOW, -1);
        }

        List<MatOfPoint> contoursYellow = new ArrayList<>();
        Imgproc.findContours(frameTemp, contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        //Imgproc.drawContours(input, contoursYellow, -1, RED, 2);
        for(MatOfPoint cont : contoursYellow) {
            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(cont);
            if (rect.area() > 200d) {
                Imgproc.rectangle(input, rect.tl(), rect.br(), RED, 2); // Draw rect
            }
        }

        Imgproc.rectangle(input, mask1, BLUE, 2);
        Imgproc.rectangle(input, mask2, BLUE, 2);

        frameTemp.release();
        region1.release();
        region2.release();




























































































        return input;
    }
}
