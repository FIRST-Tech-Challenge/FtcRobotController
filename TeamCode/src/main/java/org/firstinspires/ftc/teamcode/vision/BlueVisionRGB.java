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

public class BlueVisionRGB extends OpenCvPipeline {
    boolean viewportPaused = false;

    private volatile boolean[] positions;

    public Scalar x = new Scalar(87,179,232);
    public Scalar low = new Scalar(53.8, 72.3, 126.1, 0), high =new Scalar(121.8, 168.6, 201.2, 255);
    //public Scalar low = new Scalar(25,25,25), high = new Scalar(200,200,200);

    Mat mask, hierarchy = new Mat();

    Telemetry telemetry;
    public BlueVisionRGB(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Process frame
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
            int k = 50;
            x /= points.size();
            y /= points.size();
            if(y > 5*input.height()/8 && y < 3*input.height()/4 && xrange > input.width()/k && yrange > input.height()/k) {
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
        Imgproc.drawContours(input, cont2,-1, new Scalar(255,0,0), 5);
        return input;
    }

    public boolean[] getPositions() {
        return positions;
    }

}
