package org.firstinspires.ftc.teamcode.vision.dogecv;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.vision.GoldPos;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;


import java.util.ArrayList;
import java.util.List;

@Config
public class DogeCVPipeline {

    //Create the scorers used for the detector
    public MaxAreaScorer maxAreaScorer         = new MaxAreaScorer(0.01);
    public RatioScorer ratioScorer             = new RatioScorer(1.0,5);

    //Create the filters used
    public DogeCVColorFilter yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW,100);
    public DogeCVColorFilter whiteFilter  = new HSVRangeFilter(new Scalar(0,0,200), new Scalar(50,40,255));

    // Results for the detector
    private GoldPos currentOrder = GoldPos.NONE_FOUND;
    private GoldPos lastOrder    = GoldPos.NONE_FOUND;
    private boolean isFound      = false;

    // Create the mats used
    private Mat workingMat  = new Mat();
    private Mat displayMat  = new Mat();
    private Mat yellowMask  = new Mat();
    private Mat whiteMask   = new Mat();
    private Mat hiarchy     = new Mat();

    private double maxDifference = 10;

    // Cropping (Iron Reign Addition)
    public static double minY = 0, maxY = 0;
    public static boolean enableYCrop = false;

    private Size adjustedSize = new Size(640, 480);

    public Mat process(Mat input, DogeCVFinalStep finalStep) {

        // Copy input mat to working/display mats
        input.copyTo(displayMat);
        input.copyTo(workingMat);
        input.release();

        // Generate Masks
        yellowFilter.process(workingMat.clone(), yellowMask);
        whiteFilter.process(workingMat.clone(), whiteMask);


        // Blur and find the countours in the masks
        List<MatOfPoint> contoursYellow = new ArrayList<>();
        List<MatOfPoint> contoursWhite = new ArrayList<>();

        Imgproc.blur(whiteMask,whiteMask,new Size(2,2));
        Imgproc.blur(yellowMask,yellowMask,new Size(2,2));

        Imgproc.findContours(yellowMask, contoursYellow, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat,contoursYellow,-1,new Scalar(230,70,70),2);

        Imgproc.findContours(whiteMask, contoursWhite, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat,contoursWhite,-1,new Scalar(230,70,70),2);


        // Prepare to find best yellow (gold) results
        Rect   chosenYellowRect  = null;
        double chosenYellowScore = Integer.MAX_VALUE;

        MatOfPoint2f approxCurve = new MatOfPoint2f();

        for(MatOfPoint c : contoursYellow){
            MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());

            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

            //Convert back to MatOfPoint
            MatOfPoint points = new MatOfPoint(approxCurve.toArray());

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(points);

            double area = Imgproc.contourArea(c);
            double x = rect.x;
            double y = rect.y;
            double w = rect.width;
            double h = rect.height;
            Point centerPoint = new Point(x + ( w/2), y + (h/2));
            if (enableYCrop && !(centerPoint.y < maxY && centerPoint.y > minY))
                continue;
            if( area > 500){
                Imgproc.circle(displayMat,centerPoint,3,new Scalar(0,255,255),3);
                Imgproc.putText(displayMat,"Area: " + area,centerPoint,0,0.5,new Scalar(0,255,255));
            }


            double diffrenceScore = calculateScore(points);

            if(diffrenceScore < chosenYellowScore && diffrenceScore < maxDifference){
                chosenYellowScore = diffrenceScore;
                chosenYellowRect = rect;
            }

        }

        // Prepare to find best white (silver) results
        List<Rect>   choosenWhiteRect  = new ArrayList<>(2);
        List<Double> chosenWhiteScore  = new ArrayList<>(2);
        chosenWhiteScore.add(0, Double.MAX_VALUE);
        chosenWhiteScore.add(1, Double.MAX_VALUE);
        choosenWhiteRect.add(0, null);
        choosenWhiteRect.add(1, null);


        for(MatOfPoint c : contoursWhite){
            MatOfPoint2f contour2f = new MatOfPoint2f(c.toArray());

            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

            //Convert back to MatOfPoint
            MatOfPoint points = new MatOfPoint(approxCurve.toArray());

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(points);

            double diffrenceScore = calculateScore(points);

            double area = Imgproc.contourArea(c);
            double x = rect.x;
            double y = rect.y;
            double w = rect.width;
            double h = rect.height;
            Point centerPoint = new Point(x + ( w/2), y + (h/2));
            if (enableYCrop && !(centerPoint.y < maxY && centerPoint.y > minY))
                continue;
            if( area > 1000){
                Imgproc.circle(displayMat,centerPoint,3,new Scalar(0,255,255),3);
                Imgproc.putText(displayMat,"Area: " + area,centerPoint,0,0.5,new Scalar(0,255,255));
                Imgproc.putText(displayMat,"Diff: " + diffrenceScore,new Point(centerPoint.x, centerPoint.y + 20),0,0.5,new Scalar(0,255,255));
            }

            boolean good = true;
            if(diffrenceScore < maxDifference && area > 1000){

                if(diffrenceScore < chosenWhiteScore.get(0)){
                    choosenWhiteRect.set(0,rect);
                    chosenWhiteScore.set(0,diffrenceScore);
                }
                else if(diffrenceScore < chosenWhiteScore.get(1) && diffrenceScore > chosenWhiteScore.get(0)){
                    choosenWhiteRect.set(1,rect);
                    chosenWhiteScore.set(1, diffrenceScore);
                }
            }


        }

        //Draw found gold element
        if(chosenYellowRect != null){
            Imgproc.rectangle(displayMat,
                    new Point(chosenYellowRect.x, chosenYellowRect.y),
                    new Point(chosenYellowRect.x + chosenYellowRect.width, chosenYellowRect.y + chosenYellowRect.height),
                    new Scalar(255, 0, 0), 2);

            Imgproc.putText(displayMat,
                    "Gold: " + String.format("%.2f X=%.2f", chosenYellowScore, (double)chosenYellowRect.x),
                    new Point(chosenYellowRect.x - 5, chosenYellowRect.y - 10),
                    Imgproc.FONT_HERSHEY_PLAIN,
                    1.3,
                    new Scalar(0, 255, 255),
                    2);

        }
        //Draw found white elements
        for(int i=0;i<choosenWhiteRect.size();i++){
            Rect rect = choosenWhiteRect.get(i);
            if(rect != null){
                double score = chosenWhiteScore.get(i);
                Imgproc.rectangle(displayMat,
                        new Point(rect.x, rect.y),
                        new Point(rect.x + rect.width, rect.y + rect.height),
                        new Scalar(255, 255, 255), 2);
                Imgproc.putText(displayMat,
                        "Silver: " + String.format("Score %.2f ", score) ,
                        new Point(rect.x - 5, rect.y - 10),
                        Imgproc.FONT_HERSHEY_PLAIN,
                        1.3,
                        new Scalar(255, 255, 255),
                        2);
            }


        }

        if (enableYCrop) {
            Imgproc.rectangle(displayMat, new Point(0, maxY), new Point(displayMat.width(), maxY), new Scalar(225, 225, 0), 4);
            Imgproc.rectangle(displayMat, new Point(0, minY), new Point(displayMat.width(), minY), new Scalar(225, 225, 0), 4);

        }


        /* BEGIN IRON REIGN MODIFICATIONS */

        switch (finalStep) {
            case TWO_MINERALS:
                if (chosenYellowRect != null) {
                    Point center = new Point(chosenYellowRect.x + (chosenYellowRect.width / 2d), chosenYellowRect.y + (chosenYellowRect.height / 2d));
                    if (center.x >= workingMat.width() / 2) {
                        currentOrder = GoldPos.RIGHT;
                    } else {
                        currentOrder = GoldPos.MIDDLE;
                    }
                } else {
                    currentOrder = GoldPos.LEFT;
                }
                isFound = true;
                lastOrder = currentOrder;
                //Display Debug Information
                Imgproc.rectangle(displayMat, new Point(displayMat.width() / 2, 0), new Point(displayMat.width() / 2, displayMat.height()), new Scalar(225, 225, 0), 4);
                break;
            case THREE_MINERALS:
                if (chosenYellowRect != null) {
                    Point center = new Point(chosenYellowRect.x + (chosenYellowRect.width / 2d), chosenYellowRect.y + (chosenYellowRect.height / 2d));
                    if (center.x >= workingMat.width() * 2/3) {
                        currentOrder = GoldPos.RIGHT;
                    } else  if (center.x >= workingMat.width() *1/3){
                        currentOrder = GoldPos.MIDDLE;
                    } else {
                        currentOrder = GoldPos.LEFT;
                    }
                    isFound = true;
                    lastOrder = currentOrder;
                } else {
                    currentOrder = GoldPos.NONE_FOUND;
                    isFound = false;
                }
                //Display Debug Information
                Imgproc.rectangle(displayMat, new Point(displayMat.width() * 2/3, 0), new Point(displayMat.width() * 2/3, displayMat.height()), new Scalar(225, 225, 0), 4);
                Imgproc.rectangle(displayMat, new Point(displayMat.width() * 1/3, 0), new Point(displayMat.width() * 1/3, displayMat.height()), new Scalar(225, 225, 0), 4);
                break;
            case ORIGINAL_ALL_MINERALS:
                if(choosenWhiteRect.get(0) != null && choosenWhiteRect.get(1) != null  && chosenYellowRect != null){
                    int leftCount = 0;
                    for(int i=0;i<choosenWhiteRect.size();i++){
                        Rect rect = choosenWhiteRect.get(i);
                        if(chosenYellowRect.x > rect.x){
                            leftCount++;
                        }
                    }
                    if(leftCount == 0){
                        currentOrder = GoldPos.LEFT;
                    }

                    if(leftCount == 1){
                        currentOrder = GoldPos.MIDDLE;
                    }

                    if(leftCount >= 2){
                        currentOrder = GoldPos.RIGHT;
                    }
                    isFound = true;
                    lastOrder = currentOrder;

                }else{
                    currentOrder = GoldPos.NONE_FOUND;
                    isFound = false;
                }
                break;
        }
        Imgproc.putText(displayMat, "Gold Position: " + lastOrder.toString(), new Point(10, adjustedSize.height - 30), 0, 1, new Scalar(255, 255, 0), 1);
        Imgproc.putText(displayMat, "Current Track: " + currentOrder.toString(), new Point(10, adjustedSize.height - 10), 0, 0.5, new Scalar(255, 255, 255), 1);
        /* END IRON REIGN MODIFICATIONS */

        return displayMat;
    }

    public double calculateScore(Mat input){
        return maxAreaScorer.calculateScore(input) + ratioScorer.calculateScore(input);
    }


    /**
     * Is both elements found?
     * @return if the elements are found
     */
    public boolean isFound() {
        return isFound;
    }

    /**
     * Returns the current gold pos
     * @return current gold pos (UNKNOWN, LEFT, CENTER, RIGHT)
     */
    public GoldPos getCurrentOrder() {
        return currentOrder;
    }

    /**
     * Returns the last known gold pos
     * @return last known gold pos (UNKNOWN, LEFT, CENTER, RIGHT)
     */
    public GoldPos getLastOrder() {
        return lastOrder;
    }
}