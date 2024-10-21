package org.firstinspires.ftc.teamcode.drive;

import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.contourArea;
import static org.opencv.imgproc.Imgproc.drawContours;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config
public class ColorIsolationPipeline extends OpenCvPipeline {
    /*static public int hMax = 12;
    static public int sMax = 300;
    static public int lMax = 400;

    static public int hMin = 0;
    static public int sMin = 120;
    static public int lMin = 0;
    */

    public static boolean outputMode, blurry;
    public int hMax = 110, hMin=90, sMax=255, sMin=150, lMax=100, lMin=0;
    public static int hMax2, hMin2, sMax2, sMin2, lMax2, lMin2;
    public int pos = 0;

    public ColorIsolationPipeline() {

    }

    @Override
    public void init(Mat input) {
    }

    public Mat ROI, blur;
    @Override
    public Mat processFrame(Mat input) {
        //Drawing the Contours
            Mat temp = new Mat();
            Imgproc.cvtColor(input, temp, COLOR_BGR2HSV);
            Scalar low = new Scalar(hMin, sMin, lMin);
            Scalar high = new Scalar(hMax, sMax, lMax);
            Mat mask = new Mat();
            inRange(temp, low, high, mask);
            List<MatOfPoint> contours = new ArrayList<>();
            double maxArea = 0;
            Mat hierarchey = new Mat();
            ROI = mask.submat(0, mask.height(), 0, mask.width());
            blur = ROI;
            Imgproc.blur(ROI, blur, new Size(20, 20));

            Imgproc.findContours(ROI, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            Scalar low2 = new Scalar(hMin2, sMin2, lMin2);
            Scalar high2 = new Scalar(hMax2, sMax2, lMax2);
            Mat mask2 = new Mat();
            inRange(ROI, low2, high2, mask2);
            List<Double> areas = new ArrayList<>();
            int position = 0;

            contours.removeIf(c -> contourArea(c) < 100);
            for (MatOfPoint m : contours) areas.add(contourArea(m));
            //areas = areas.stream().distinct().collect(Collectors.toList());
            // above introduces edge case where two sides actually have same area, but that edgy enough im willing to roll the dice (0.05% chance)
            // should probably do something else tho
            List<Double> sortedAreas = new ArrayList<>();
            for (double m : areas) sortedAreas.add(m);


            Collections.sort(sortedAreas);
            Collections.reverse(sortedAreas);



            if (areas.size() > 2) {
                List<MatOfPoint> conts = new ArrayList<MatOfPoint>();
                drawContours(input, conts, -1, new Scalar(255, 0, 255), 2, Imgproc.LINE_8, new Mat(), 2, new Point());
            }

        if (!blur.empty() && blurry) return mask2;

        return (ROI.empty()) ? input : ((outputMode) ? ROI : input);

    }

}