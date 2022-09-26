package org.firstinspires.ftc.teamcode.Pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SkystoneDetectorPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    // There are only three possible positions for the skystone: left, right, or not found
    public enum Location{
        LEFT,
        RIGHT,
        NOT_FOUND
    }
    private Location skystoneLocation;

    // ROI --> region of interest
    // We have two regions we want to focus on because our camera has two skystones in view
    static final Rect LEFT_ROI = new Rect(
        new Point(60, 35),
        new Point(120, 75)
    );
    static final Rect RIGHT_ROI = new Rect(
        new Point(140,35),
        new Point(200,75)
    );
    static final double PERCENTAGE_COLOR_THRESHOLD = 0.4; // 40% of yellow --> stone detected


    public SkystoneDetectorPipeline(Telemetry telemetry){this.telemetry = telemetry;}
    @Override
    public Mat processFrame(Mat input){
        // It is difficult to specify color ranges using RGB, so people use HSRV or YCrCb
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); // HSV uses a color range of 180

        // new Scalar(<Hues>, <Saturation>, <Value>) respectively
        Scalar minAcceptedHSRV = new Scalar(23, 50, 70);
        Scalar maxAcceptedHSRV = new Scalar(32, 255, 255);

        // the first mat argument is our source matrix
        // the second mat argument is our original matrix being used as a destination matrix
        Core.inRange(mat, minAcceptedHSRV, maxAcceptedHSRV, mat);

        // Becase we have to different regions to focus on, we are going to make two different submats,
        // or smaller matrices inside of the original one
        Mat leftMat = mat.submat(LEFT_ROI);
        Mat rightMat = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(leftMat).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(rightMat).val[0] / RIGHT_ROI.area() / 255;

        leftMat.release();
        leftMat.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(leftMat).val[0]);

        telemetry.addData("Left percentage in range", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage in range", Math.round(rightValue * 100) + "%");

        boolean stoneLeft = leftValue > PERCENTAGE_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENTAGE_COLOR_THRESHOLD;

        // Using our enum made below the class declaration
        if(stoneLeft && stoneRight){
            skystoneLocation = skystoneLocation.NOT_FOUND;
            telemetry.addData("Skystone location", "NOT_FOUND");
        }
        if(stoneLeft){
            skystoneLocation = skystoneLocation.RIGHT;
            telemetry.addData("Skystone location", "RIGHT");
        }
        else {
            skystoneLocation = skystoneLocation.LEFT;
            telemetry.addData("Skystone location", "LEFT");
        }
        telemetry.update();

        // We can draw rectangles to visualize the location of the stones
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone  = new Scalar(255, 0, 0); // Red means normal stone (what we ignore)
        Scalar colorSkystone = new Scalar(0, 255, 0); // Green means skystone (what we want)

        // <location = skystoneLocation.LEFT?> checks to see if the skystone is on the left
        // If it is, pass through the skystone color into the .rectangle() method, and if not, pass through the stone color
        Imgproc.rectangle(mat, LEFT_ROI, skystoneLocation == skystoneLocation.LEFT? colorSkystone : colorStone, 2);
        Imgproc.rectangle(mat, RIGHT_ROI, skystoneLocation == skystoneLocation.RIGHT? colorSkystone : colorStone, 2);

        // Now we can return the matrix of pixel values (and our rectangle we just made) since they have been analyzed for skystone locations
        return mat;
    }

    public Location getSkystoneLocation(){
        return skystoneLocation;
    }
}
