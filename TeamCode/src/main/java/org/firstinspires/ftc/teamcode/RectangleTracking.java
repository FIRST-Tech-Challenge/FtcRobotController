package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

class RectangleTracking extends OpenCvPipeline
{
    Mat imgHSV = new Mat();
    Mat thresholdMat0 = new Mat();
    Mat thresholdMat1 = new Mat();
    Mat thresholdMat = new Mat();
    Mat HSLchan0Mat = new Mat();
    Mat HSLchan1Mat = new Mat();
    Mat thresholdMatS = new Mat();
    Mat thresholdMatH = new Mat();
    Mat contoursOnFrameMat = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();
    List<MatOfPoint> bigContours = new ArrayList<>();
    MatOfPoint bigContour = new MatOfPoint();
    int numContoursFound;

    Size viewportSize;

    double hHSVAverage = 0;
    double sHSVAverage = 0;
    double vHSVAverage = 0;

    int rangeAccuracyH = 10;
    int rangeAccuracyS = 30;

    double[] rectangleCoordinates = {0, 0};

    Telemetry telemetry;
    Gamepad gamepad;

    public RectangleTracking(Telemetry telemetry, Gamepad gamepad)
    {
        this.telemetry = telemetry;
        this.gamepad = gamepad;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        //Add a Gausian Blur to make more smooth the contours
        Imgproc.GaussianBlur(input, input, new Size(5, 5), 20);

        //Convert image from RGB to HSV
        Imgproc.cvtColor(input, imgHSV, Imgproc.COLOR_RGB2HSV);

        //Assing viewportSize variable to viewport size in pixels
        viewportSize = input.size();

        //Getting certainPixels
        double[] pixel1 = imgHSV.get((int) viewportSize.height/2, (int) viewportSize.width/2);
        double[] pixel2 = imgHSV.get((int) viewportSize.height/2+1, (int) viewportSize.width/2);
        double[] pixel3 = imgHSV.get((int) viewportSize.height/2+1, (int) viewportSize.width/2+1);
        double[] pixel4 = imgHSV.get((int) viewportSize.height/2, (int) viewportSize.width/2+1);

        //Getting Average HSV Values only when Stage=THRESHOLD
        if(gamepad.options) {
            hHSVAverage = (pixel1[0] + pixel2[0] + pixel3[0] + pixel4[0]) / 4;
            sHSVAverage = (pixel1[1] + pixel2[1] + pixel3[1] + pixel4[1]) / 4;
            vHSVAverage = (pixel1[2] + pixel2[2] + pixel3[2] + pixel4[2]) / 4;
        }

        contoursList.clear();

        //Threshold
        Imgproc.cvtColor(input, imgHSV, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(imgHSV, HSLchan0Mat, 0);
        Core.extractChannel(imgHSV, HSLchan1Mat, 1);
        Imgproc.threshold(HSLchan0Mat, thresholdMat0, hHSVAverage - rangeAccuracyH, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(HSLchan0Mat, thresholdMat1, hHSVAverage + rangeAccuracyH, 255, Imgproc.THRESH_BINARY_INV);
        Core.bitwise_and(thresholdMat0,thresholdMat1, thresholdMatH);
        Imgproc.threshold(HSLchan1Mat, thresholdMat0, sHSVAverage - rangeAccuracyS, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(HSLchan1Mat, thresholdMat1, sHSVAverage + rangeAccuracyS, 255, Imgproc.THRESH_BINARY_INV);
        Core.bitwise_and(thresholdMat0,thresholdMat1, thresholdMatS);

        Core.bitwise_and(thresholdMatH,thresholdMatS, thresholdMat);

        //Find Contours
        Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        numContoursFound = contoursList.size();
        input.copyTo(contoursOnFrameMat);

        //Find the biggest contour
        int contourIdx = 0;
        for(MatOfPoint c : contoursList) {
            if(contoursList.indexOf(c) == 0) {
                bigContour = c;
            }

            double contourArea = Imgproc.contourArea(contoursList.get(contourIdx));
            if(contourArea > Imgproc.contourArea(bigContour)) bigContour = contoursList.get(contourIdx);
            contourIdx++;
        }

        bigContours.clear();

        //Add Biggest Contour to a list
        bigContours.add(bigContour);

        //Draw the biggest Contour
        Imgproc.drawContours(contoursOnFrameMat, bigContours, -1, new Scalar(0 , 255, 0), 2, 8);

        //Draw Bounding Rectangle

        Imgproc.rectangle(contoursOnFrameMat, Imgproc.boundingRect(bigContour).tl(), Imgproc.boundingRect(bigContour).br(), new Scalar(255,255,0),2);

        int crossThickness = 1;
        double crossSize = 20;

        double[] centerCrossPosition = {0, 0};
        centerCrossPosition[0] = this.viewportSize.width / 2;
        centerCrossPosition[1] = this.viewportSize.height / 2;

        telemetry.addData(">", "Contour Area: " + Imgproc.contourArea(bigContour));

        this.createCross(centerCrossPosition, 1, 12, new Scalar(0, 127, 100), contoursOnFrameMat);

        rectangleCoordinates[0] = (Imgproc.boundingRect(bigContour).br().x - Imgproc.boundingRect(bigContour).tl().x)/2 + Imgproc.boundingRect(bigContour).tl().x;
        rectangleCoordinates[1] = (Imgproc.boundingRect(bigContour).br().y - Imgproc.boundingRect(bigContour).tl().y)/2 + Imgproc.boundingRect(bigContour).tl().y;

        this.createCross(rectangleCoordinates, 1, 8, new Scalar(255, 255, 0), contoursOnFrameMat);

        telemetry.addData(">", "Element's X:" + this.getElementsAnalogCoordinates()[0]);
        telemetry.addData(">", "Element's Y:" + this.getElementsAnalogCoordinates()[1]);
        telemetry.update();

        return contoursOnFrameMat;

    }

    public void createCross(double[] pos, int thickness, double sizeOfCross, Scalar color, Mat matToRender) {
        Imgproc.line(
                matToRender,
                new Point(pos[0] - sizeOfCross, pos[1]),
                new Point(pos[0] + sizeOfCross, pos[1]),
                color,
                thickness
        );

        Imgproc.line(
                matToRender,
                new Point(pos[0], pos[1] - sizeOfCross),
                new Point(pos[0], pos[1] + sizeOfCross),
                color,
                thickness
        );
    }

    public double[] getElementsAnalogCoordinates()
    {
        double[] centerOfScreen = {0, 0};
        double[] rectangleCoordinatesMapped = {0, 0};
        centerOfScreen[0] = this.viewportSize.width/2;
        centerOfScreen[1] = this.viewportSize.height/2;
        rectangleCoordinatesMapped[0] = (rectangleCoordinates[0] - centerOfScreen[0]) / (this.viewportSize.width / 2 + 0.000000000000000000001);
        rectangleCoordinatesMapped[1] = (centerOfScreen[1] - rectangleCoordinates[1]) / (this.viewportSize.height / 2 + 0.000000000000000000001);

        return rectangleCoordinatesMapped;
    }

    public int getNumContoursFound()
    {
        return numContoursFound;
    }
}
