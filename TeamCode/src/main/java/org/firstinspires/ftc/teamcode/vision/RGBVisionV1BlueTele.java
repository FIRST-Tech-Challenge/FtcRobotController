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

public class RGBVisionV1BlueTele extends OpenCvPipeline {
    boolean viewportPaused = false;

    private volatile boolean[] positions;

    public Scalar x = new Scalar(87,179,232);
    public Scalar low = new Scalar(35, 111.9, 160.1), high =new Scalar(106.3, 188.4, 201.2);

    Mat mask, hierarchy = new Mat();

    Telemetry telemetry;
    public RGBVisionV1BlueTele(Telemetry telemetry) {
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
        telemetry.addData("ContourN",cont.size());
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
            telemetry.addLine("("+x+","+y+")");
            if(y > 3*input.height()/4 && xrange > input.width()/k && yrange > input.height()/k) {
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
        Imgproc.drawContours(input ,cont2,-1, new Scalar(255,0,0), 5);
        telemetry.addData("left",positions[0]);
        telemetry.addData("middle",positions[1]);
        telemetry.addData("right",positions[2]);
        telemetry.update();
        return input;
    }
    public int getPosition() {
        if(positions[0] && positions[1]) {
            return 3;
        }else if(positions[1] && positions[2]) {
            return 1;
        }else {
            return 2;
        }
    }

}
