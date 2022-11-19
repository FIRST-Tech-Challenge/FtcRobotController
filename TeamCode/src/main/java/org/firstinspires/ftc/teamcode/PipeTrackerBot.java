package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;


public class PipeTrackerBot extends OpenCvPipeline {

    Telemetry telemetry;
    Mat mat = new Mat();
    static double AvgX = 0;
    static double AvgY = 0;
    static int l = 0;
    static int o = 0;
    static int p = 0;

    static int targetBoxes = 0;
    static int currentBoxes = 0;
    static int differenceBoxes = 0;
    static int counter = 0;
    static int lastBoxes = 0;
    static int timeBoxesDifference = 0;

//







    public PipeTrackerBot(Telemetry t){
        telemetry = t;
    }


    @Override
    public Mat processFrame(Mat input) {
        mat = input;

        String file = "C:\\Images\\rainbow.jpg";
//        Mat src = Imgcodecs.imread(file);


        /** New Algorithm (dividing image up)
         *
         *
         */


        Mat HSVsrc = new Mat();
        Mat YCrCbSrc = new Mat();


        //Equalizing HSV Image via Value
        Imgproc.cvtColor(mat,HSVsrc,Imgproc.COLOR_BGR2HSV);
        ArrayList<Mat> HSVList = new ArrayList<>();
        Core.split(HSVsrc,HSVList);
        Imgproc.equalizeHist(HSVList.get(2),HSVList.get(2));
        Core.merge(HSVList,mat);
        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_HSV2BGR);

        HSVsrc.release();





        Mat grayRainbow = new Mat();
        Mat hsvRainbow = new Mat();
        Mat maskRainbow = new Mat();
        Imgproc.cvtColor(mat, hsvRainbow, Imgproc.COLOR_BGR2HSV);
        Core.inRange(hsvRainbow,  new Scalar(83, 130 , 50), new Scalar(96, 245, 245), maskRainbow);


        int gridX = 20;
        int gridY = 20;
        double percentThreshold = 70;


        int XResolution = maskRainbow.cols();
        int YResolution = maskRainbow.rows();

        int amountBoxes = gridX * gridY;
        double boxWidth = XResolution / gridX;
        double boxHeight = YResolution / gridY;
        int amountXRelationships = (gridX - 1) * (gridY);
        int amountYRelationships = (gridY - 1) * (gridX);
        double TotalX = 0;
        double TotalY = 0;

        Scalar white = new Scalar(0, 0, 0); // In grey scale
        Scalar grey = new Scalar(75, 0, 0); // In grey scale
        Scalar red = new Scalar(0, 0, 255); // in BGR
        Scalar blue = new Scalar(255, 0, 0); // in BGR
        Scalar green = new Scalar(0, 255, 0); // in BGR


        int[] XBoxesLeft = new int[gridX];
        int[] YBoxesTop = new int[gridY];
        Rect[] Boxes = new Rect[amountBoxes];
        int[] boxXCenters = new int[amountBoxes];
        int[] boxYCenters = new int[amountBoxes];
        Mat[] subMats = new Mat[amountBoxes];
        int[] regionSums = new int[amountBoxes];
        double[] percentColor = new double[amountBoxes];
        boolean[] hasColor = new boolean[amountBoxes];
        int[] identifiedBoxes = new int[amountBoxes];
        boolean[] xRelationship = new boolean[amountXRelationships];
        boolean[] yRelationship = new boolean[amountYRelationships];
        int[] identifiedXRelationships = new int[amountXRelationships];
        int[] identifiedYRelationships = new int[amountYRelationships];

        int u = 0;




        //Setting the initial x positions for the boxes
        for (int i = 0; i < gridX; i++) {
            XBoxesLeft[i] = (int) (i * boxWidth);
        }

        //Setting the initial y positions for the boxes
        for (int i = 0; i < gridY; i++) {
            YBoxesTop[i] = (int) (i * boxHeight);
        }

        //Creating all the rectangles  for the submats
        for (int i = 0; i < gridX; i++) {
            for (int j = 0; j < gridY; j++) {
                Boxes[gridY * i + j] = new Rect(new Point(XBoxesLeft[i], YBoxesTop[j]), new Point(XBoxesLeft[i] + boxWidth, YBoxesTop[j] + boxHeight));
                boxXCenters[gridY * i + j] = (int) ((0.5 * boxWidth) + (i * boxWidth));
                boxYCenters[gridY * i + j] = (int) ((0.5 * boxHeight) + (j * boxHeight));
            }
        }


        //Creating all the submats
        for (int i = 0; i < amountBoxes; i++) {
            subMats[i] = maskRainbow.submat(Boxes[i]);
        }


        //Calculating how many pixels light up in a particular box
        for (int i = 0; i < amountBoxes; i++) {
            regionSums[i] = Core.countNonZero(subMats[i]);

        }

        //Calculating percentage based on regions sums
        for (int i = 0; i < amountBoxes; i++) {
            percentColor[i] = (regionSums[i] / Boxes[i].area()) * 100;
        }

        //Determining which boxes have the correct amount of color in them
        for (int i = 0; i < amountBoxes; i++) {
            if (percentColor[i] > percentThreshold) {
                hasColor[i] = true;
            } else {
                hasColor[i] = false;
            }
        }

        //Drawing the rectangles
//            for (int i = 0; i < amountBoxes; i++) {
//                Scalar rectGreyColor = white;
//                Scalar rectBGRColor = blue;
//                if (hasColor[i] == true) {
//                    rectGreyColor = grey;
//                    rectBGRColor = green;
//                }
//                Imgproc.rectangle(mat, Boxes[i], rectBGRColor, 2);
//                Imgproc.rectangle(maskRainbow, Boxes[i], rectGreyColor, 2); //Only adjust the first channel in this scalar as it's a greyscale image
//
//            }

        //  Draw out all the center points
        for (int i = 0; i < amountBoxes; i++) {
            Imgproc.ellipse(maskRainbow, new RotatedRect(new Point(boxXCenters[i], boxYCenters[i]), new Size(2, 2), 180), new Scalar(50, 0, 0), Imgproc.FILLED);
        }

        // Listing which boxes have been found
        for (int i = 0; i < amountBoxes; i++) {
            if (hasColor[i] == true) {
                identifiedBoxes[l] = i;
                l++;
            } else {

            }
        }

        //Find average point
        for (int i = 0; i < l; i++) {
            TotalX += boxXCenters[identifiedBoxes[i]];
            TotalY += boxYCenters[identifiedBoxes[i]];

        }

        //Determining which horizontal relationships exist
        for (int i = 0; i < amountXRelationships; i++) {
            if (hasColor[i] == true && hasColor[i + gridY] == true) {
                xRelationship[i] = true;
            } else {
                xRelationship[i] = false;
            }
        }



        for(int i = 0; i < gridX; i++){
            for(int j = 0; j < (gridY-1); j++){
                if(hasColor[j+(i*gridY)] == true && hasColor[j+(i*gridY)+1]){
                    yRelationship[j+(i*(gridY-1))] = true;
                }else{
                    yRelationship[j+(i*(gridY-1))] = false;
                }
            }
        }

//

        //Draw horizontal "x" relationships

        for (int i = 0; i < amountXRelationships; i++) {

            if (xRelationship[i] == true) {
                int horizontalP1 = boxXCenters[i] + (int) ((0.25) * (boxHeight));
                int horizontalP2 = boxXCenters[i] + (int) ((0.75) * (boxHeight));
                Imgproc.line(mat, new Point(horizontalP1, boxYCenters[i]), new Point(horizontalP2, boxYCenters[i]), red, 2);
            } else {

            }
        }


        //Draw vertical "y" relationships
        for(int i = 0; i < gridX; i++)
            for (int j = 0; j < (gridY-1); j++) {
                if (yRelationship[j+(i*(gridY-1))] == true) {
                    int verticalP1 = boxYCenters[j+i*gridY] + (int) ((0.25) * (boxHeight));
                    int verticalP2 = boxYCenters[j+i*gridY] + (int) ((0.75) * (boxHeight));
                    Imgproc.line(mat, new Point(boxXCenters[j+i*gridY], verticalP1), new Point(boxXCenters[j+i*gridY], verticalP2), red, 2);

                } else {

                }

            }

        // Listing which "x" Relationships have been found
        for (int i = 0; i < amountXRelationships; i++) {
            if (xRelationship[i] == true) {
                identifiedXRelationships[o] = i;
                o++;
            } else {

            }
        }

        // Listing which "y" Relationships have been found
        for (int i = 0; i < amountYRelationships; i++) {
            if (yRelationship[i] == true) {
                identifiedYRelationships[p] = i;
                p++;
            } else {

            }
        }

        int[][] objects = new int[amountBoxes/2][amountBoxes];
        for(int i = 0; i < amountXRelationships; i++){


        }

        currentBoxes = l;
        if(counter == 0){
            lastBoxes = currentBoxes;
            counter = 1;
        }

        if(counter < 21){
            counter++;
        }

        if(counter == 20){
            targetBoxes = currentBoxes;

        }

        differenceBoxes = targetBoxes - currentBoxes;



        AvgX = TotalX / l;
        AvgY = TotalY / l;
        Imgproc.ellipse(maskRainbow, new RotatedRect(new Point(AvgX, AvgY), new Size(10, 10), 180), new Scalar(50, 0, 0), Imgproc.FILLED);
        Imgproc.ellipse(mat, new RotatedRect(new Point(AvgX, AvgY), new Size(10, 10), 180), new Scalar(0, 0, 255), Imgproc.FILLED);



        telemetry.addLine("percentColor: " + Arrays.toString(percentColor));
        telemetry.addLine("hasColor: " + Arrays.toString(hasColor));
        telemetry.addLine("identifiedBoxes: " + Arrays.toString(identifiedBoxes));
        telemetry.addLine("identifiedXRelationships: " + Arrays.toString(identifiedXRelationships));
        telemetry.addLine("identifiedYRelationships: " + Arrays.toString(identifiedYRelationships));

        telemetry.addLine("Amount of Detected Boxes (l): " + l);
        telemetry.addLine("o: " + o);
        telemetry.addLine("p: " + p);

        telemetry.addLine("TotalX: " + TotalX);
        telemetry.addLine("TotalY: " + TotalY);
        telemetry.addLine("avgX: " + AvgX);
        telemetry.addLine("avgY: " + AvgY);




        l = 0;
        o = 0;
        p = 0;
        TotalX = 0;
        TotalY = 0;


        timeBoxesDifference = currentBoxes - lastBoxes;
        lastBoxes = currentBoxes;


        grayRainbow.release();
        maskRainbow.release();
        hsvRainbow.release();



        return mat;



    }


    public static int getAmountDetectedBoxes () {return currentBoxes;}
    public static double getAvgXBoxes () {return AvgX;}
    public static double getAvgYBoxes () {return AvgY;}
    public static double getDifferenceBoxes () {return differenceBoxes;}
    public static double getTargetBoxes () {return targetBoxes;}
    public static double getLastCurrentBoxesDifference () {return timeBoxesDifference;}





}