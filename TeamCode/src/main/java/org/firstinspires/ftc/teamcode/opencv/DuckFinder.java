package org.firstinspires.ftc.teamcode.opencv;

import androidx.annotation.NonNull;

import org.apache.commons.math3.fraction.Fraction;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class DuckFinder extends OpenCvPipeline {
    ArrayList<MatOfPoint> duckContours = new ArrayList<>();
    ArrayList<Point> duckCenters = new ArrayList<>();
    Rect duckRect;


    // Camera Settings

    protected double centerX;
    protected double centerY;

    protected int imageWidth;
    protected int imageHeight;

    private double cameraPitchOffset;
    private double cameraYawOffset;

    private double fov;
    private double horizontalFocalLength;
    private double verticalFocalLength;

    int index0 = 0;
    boolean duckOnScreen = false;

    public ArrayList<Point> getDuckCenters() {
        return duckCenters;
    }

    public DuckFinder(double fov, double cameraPitchOffset, double cameraYawOffset) {
        super();
        this.fov = fov;
        this.cameraPitchOffset = cameraPitchOffset;
        this.cameraYawOffset = cameraYawOffset;
    }

    public DuckFinder(double fov) {
        this(fov, 0, 0);
    }

    @Override
    public void init(Mat mat) {
        super.init(mat);

        imageWidth = mat.width();
        imageHeight = mat.height();

        // pinhole model calculations
        double diagonalView = Math.toRadians(this.fov);
        Fraction aspectFraction = new Fraction(this.imageWidth, this.imageHeight);
        int horizontalRatio = aspectFraction.getNumerator();
        int verticalRatio = aspectFraction.getDenominator();
        double diagonalAspect = Math.hypot(horizontalRatio, verticalRatio);
        double horizontalView = Math.atan(Math.tan(diagonalView / 2) * (horizontalRatio / diagonalAspect)) * 2;
        double verticalView = Math.atan(Math.tan(diagonalView / 2) * (verticalRatio / diagonalAspect)) * 2;
        horizontalFocalLength = this.imageWidth / (2 * Math.tan(horizontalView / 2));
        verticalFocalLength = this.imageHeight / (2 * Math.tan(verticalView / 2));
    }

    public Point getFirstCenter() {
        if(duckOnScreen && duckCenters.size() > index0) {
            return duckCenters.get(index0);
        } else {
            return null;
        }
    }

    // A pipeline that finds and draws contours around the the ducks in the frame
    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lower = new Scalar(22, 60, 50);
        Scalar upper = new Scalar(33, 255, 255);
//        Scalar lower = new Scalar(0, 0, 0);
//        Scalar upper = new Scalar(255, 255, 255);
        Core.inRange(mat, lower, upper, mat);
        Imgproc.GaussianBlur(mat, mat, new org.opencv.core.Size(9, 9), 0);

        duckContours.clear();
        Imgproc.findContours(mat, duckContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        if(duckContours.size() > 0) {
            index0 = duckCenters.size();
            duckOnScreen = true;
        } else {
            duckOnScreen = false;
        }
        for (int i = 0; i < duckContours.size(); i++) {
            Rect rect = Imgproc.boundingRect(duckContours.get(i));
            if (rect.width > 10 && rect.height > 10) {
                if (duckRect == null) {
                    duckRect = rect;
                } else if (rect.area() > duckRect.area()) {
                    duckRect = rect;
                }
            }
        }
        if (duckRect != null){
            Imgproc.rectangle(input, duckRect, new Scalar(0, 255, 0));
            duckCenters.add(getCenterofRect(duckRect));
        }

        mat.release();
        return input;
    }

    public double calculateYaw(double offsetCenterX) {
        Rect currentRect = duckRect;
        double duckCenterX = getCenterofRect(currentRect).x;

        return Math.atan((duckCenterX - offsetCenterX) / horizontalFocalLength
        );
    }

    public Point getCenterofRect(@NonNull Rect rect) {
        return new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
    }
}