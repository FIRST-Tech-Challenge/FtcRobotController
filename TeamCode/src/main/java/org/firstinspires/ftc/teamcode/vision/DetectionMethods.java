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

import java.util.ArrayList;
import java.util.List;

public class DetectionMethods {
    public static ArrayList<VisionObject> detectYCrCb(Mat RGBin, Scalar low, Scalar high, double minX, double maxX, double minY, double maxY, double minSize, double maxSize, String kind) {
        Mat mask = new Mat(RGBin.rows(), RGBin.cols(), CvType.CV_8UC1); // Create new Mat so we don't edit the old one
        Imgproc.cvtColor(RGBin, mask, Imgproc.COLOR_RGB2YCrCb); // Convert to YCrCb
        Core.inRange(mask, low, high, mask); // Mask the copied frame so that it shows the regions we want
        Imgproc.GaussianBlur(mask, mask, new Size(5.0, 15.0), 0.00); // Blur to improve detection accuracy
        ArrayList<MatOfPoint> cont = new ArrayList<MatOfPoint>(); // Create an ArrayList to store contours of blobs of color
        Imgproc.findContours(mask, cont, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE); // Find those blobs and store them in cont
        ArrayList<VisionObject> v = new ArrayList<>(); // Create an ArrayList of VisionThings to store the objects we like
        for(MatOfPoint m : cont) { // Iterate through each contour
            int x = 0, y = 0, xmax = Integer.MIN_VALUE, xmin = Integer.MAX_VALUE, ymax = Integer.MIN_VALUE, ymin = Integer.MAX_VALUE;
            // Find average x and y values, plus the x and y-ranges for each contour
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
            } // Omitted for sake of brevity
            int xrange = xmax-xmin, yrange = ymax-ymin;
            x /= points.size();
            y /= points.size();
            // Check whether the contour appears in the correct location and is the correct size
            // The size/position factors are %s to ensure that we get the same result, no matter the resolution of incoming frames
            if(within(x,mask.width()*minX,mask.width()*maxX) && within(y,mask.height()*minY,mask.height()*maxY)
                    && within(Math.sqrt(Math.pow(xrange,2)+Math.pow(yrange,2)),
                    Math.sqrt(Math.pow(mask.width(),2)+Math.pow(mask.height(),2))*minSize,
                    Math.sqrt(Math.pow(mask.width(),2)+Math.pow(mask.height(),2))*maxSize)) {
                // Create a new VisionThing with the contour's information
                v.add(new VisionObject(x,y,xrange,yrange,kind));
            }
        }
        mask.release(); // Release the Mat to prevent memory overflow
        return v; // Return the ArrayList of VisionThings to the next method
    }

    public static void detectYCrCbTelemetry(Mat RGBin, Scalar low, Scalar high, double minX, double maxX, double minY, double maxY, double minSize, double maxSize, String kind, Telemetry telemetry) {
        Mat mask = new Mat(RGBin.rows(), RGBin.cols(), CvType.CV_8UC1); // Create new Mat so we don't edit the old one
        Imgproc.cvtColor(RGBin, mask, Imgproc.COLOR_RGB2YCrCb); // Convert to YCrCb
        Core.inRange(mask, low, high, mask); // Mask the copied frame so that it shows the regions we want
        Imgproc.GaussianBlur(mask, mask, new Size(5.0, 15.0), 0.00); // Blur to improve detection accuracy
        ArrayList<MatOfPoint> cont = new ArrayList<MatOfPoint>(); // Create an ArrayList to store contours of blobs of color
        Imgproc.findContours(mask, cont, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE); // Find those blobs and store them in cont
        ArrayList<VisionObject> v = new ArrayList<>(); // Create an ArrayList of VisionThings to store the objects we like
        for(MatOfPoint m : cont) { // Iterate through each contour
            int x = 0, y = 0, xmax = Integer.MIN_VALUE, xmin = Integer.MAX_VALUE, ymax = Integer.MIN_VALUE, ymin = Integer.MAX_VALUE;
            // Find average x and y values, plus the x and y-ranges for each contour
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
            } // Omitted for sake of brevity
            int xrange = xmax-xmin, yrange = ymax-ymin;
            x /= points.size();
            y /= points.size();
            double s = Math.sqrt(Math.pow(xrange,2)+Math.pow(yrange,2))/Math.sqrt(Math.pow(mask.width(),2)+Math.pow(mask.height(),2));
            telemetry.addLine("Contour occuring at ("+x+","+y+") with magsize "+s);
            // Check whether the contour appears in the correct location and is the correct size
            // The size/position factors are %s to ensure that we get the same result, no matter the resolution of incoming frames
            if(within(x,mask.width()*minX,mask.width()*maxX) && within(y,mask.height()*minY,mask.height()*maxY)
                    && within(Math.sqrt(Math.pow(xrange,2)+Math.pow(yrange,2))/Math.sqrt(Math.pow(mask.width(),2)+Math.pow(mask.height(),2)),
                    minSize,maxSize)) {
                // Create a new VisionThing with the contour's information
                v.add(new VisionObject(x,y,xrange,yrange,kind));
                telemetry.addLine("Contour is good");
            }
        }
        mask.release(); // Release the Mat to prevent memory overflow
    }

    public static boolean within(double check, double low, double high) {
        return check > low && check < high;
    }
}
