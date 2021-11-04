package org.firstinspires.ftc.teamcode.vision;

import org.checkerframework.checker.units.qual.A;
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
    // Takes Mat and returns the positions and sizes of contours that might be tape.
    public ArrayList<MatOfPoint> detectTape(Mat input, Scalar low, Scalar high) {
        Mat mask, hierarchy = new Mat();

        Core.inRange(input, low, high, mask);

        Imgproc.GaussianBlur(mask, mask, new Size(5.0, 15.0), 0.00);

        ArrayList<MatOfPoint> cont = new ArrayList<MatOfPoint>();

        Imgproc.findContours(mask, cont, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        ArrayList<MatOfPoint> cont2 = new ArrayList<>();

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
            int k = 50;
            x /= points.size();
            y /= points.size();
            if(y > 5*input.height()/8 && y < 3*input.height()/4 && xrange > input.width()/k && yrange > input.height()/k) {
                cont2.add(m);
            }
        }
        return cont2;
    }
}
