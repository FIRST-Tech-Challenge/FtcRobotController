package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BlueVisionYCbCr extends OpenCvPipeline {
    boolean viewportPaused = false;

    private volatile boolean[] positions;

    public Scalar x = new Scalar(87,179,232);
    public Scalar low = new Scalar(0, 87.8, 138.8, 0), high =new Scalar(255, 117.6, 194.1, 255);
    //public Scalar low = new Scalar(25,25,25), high = new Scalar(200,200,200);

    Mat mask, hierarchy = new Mat();

    Telemetry telemetry;
    public BlueVisionYCbCr(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Process frame
        Imgproc.cvtColor(input,input,Imgproc.COLOR_RGB2YCrCb);

        mask = new Mat(input.rows(), input.cols(), CvType.CV_8UC1);

        Core.inRange(input, low, high, mask);

        Imgproc.GaussianBlur(mask, mask, new Size(5.0, 15.0), 0.00);

        ArrayList<MatOfPoint> cont = new ArrayList<MatOfPoint>();

        Imgproc.findContours(mask, cont, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        telemetry.addData("numContours",cont.size());
        positions = new boolean[]{false, false, false};
        // Return information

        ArrayList<MatOfPoint> cont2 = new ArrayList<MatOfPoint>();

        for(MatOfPoint m : cont) {
            int x = 0, y = 0, xmax = Integer.MIN_VALUE, xmin = Integer.MAX_VALUE, ymax = Integer.MIN_VALUE, ymin = Integer.MAX_VALUE;
            // Find average value of point
            List<Point> points = m.toList();
            for(Point p : points) {
                x += p.x;
                y += p.y;
                if(p.x>xmax) {
                    xmax = (int) p.x;
                } if(p.x<xmin) {
                    xmin = (int) p.x;
                } if(p.y>ymax) {
                    ymax = (int) p.y;
                } if(y<ymin) {
                    ymin = (int) p.y;
                }
            }
            int xrange = xmax-xmin;
            int yrange = ymax-ymin;
            int k_low = 40;
            int k_high = 20;
            x /= points.size();
            y /= points.size();
            if(within(y,5*input.height()/8,3*input.height()/4)
                    && within(xrange,input.width()/k_low,input.width()/k_high)
                    && within(yrange ,input.height()/k_low,input.height()/k_high)) {
                cont2.add(m);
                if(x < 3*input.width()/8) {
                    positions[0] = true;
                }
                else if(x < 5*input.width()/8) {
                    positions[1] = true;
                }
                else{
                    positions[2] = true;
                }
            }
        }
        telemetry.addData("validContours",String.valueOf(cont2.size()));
        telemetry.addData("left",positions[0]);
        telemetry.addData("middle",positions[1]);
        telemetry.addData("right",positions[2]);
        telemetry.update();
        Imgproc.drawContours(input, cont2,-1, new Scalar(255,0,0), 5);
        return input;
    }

    public boolean[] getPositions() {
        return positions;
    }

    public static boolean within(double check, double low, double high) {
        return check > low && check < high;
    }

}
