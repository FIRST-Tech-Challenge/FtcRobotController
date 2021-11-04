package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class ObjectDetectionMethods {
    // Takes a contour and returns x, y, xrange, and yrange
    public static int[] getPositionInformation(MatOfPoint m) {
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
        int k = 50;
        x /= points.size();
        y /= points.size();
        return new int[]{x,xrange,y,yrange};
    }

    // Takes Mat and returns the positions and sizes of contours that might be tape.
    // colorRange = {low, high}; positionRangeProp = {xLow, xHigh, yLow, yHigh} as proportions from 0-1; sizeRangeProp = {xSize,ySize}
    public static ArrayList<MatOfPoint> detect(Mat input, Scalar[] colorRange, double[] positionRangeProp, double[] sizeRangeProp) {
        Core.inRange(input, colorRange[0], colorRange[1], input);

        Imgproc.GaussianBlur(input, input, new Size(5.0, 15.0), 0.00);

        ArrayList<MatOfPoint> cont = new ArrayList<MatOfPoint>();

        Imgproc.findContours(input, cont, input, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        input.release();

        ArrayList<MatOfPoint> cont2 = new ArrayList<>();

        for(MatOfPoint m : cont) {
            int[] pos = getPositionInformation(m);
            if(pos[0] > positionRangeProp[0]*input.width() && pos[0] < positionRangeProp[1]*input.width() &&
                    pos[2] > positionRangeProp[2]*input.height() && pos[2] < positionRangeProp[3]*input.height() &&
                    pos[1] > input.width()/sizeRangeProp[0] && pos[3] > input.height()/sizeRangeProp[1]) {
                cont2.add(m);
            }
        }
        return cont2;
    }
}
