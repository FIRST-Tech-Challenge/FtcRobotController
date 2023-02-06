package org.firstinspires.ftc.teamcode;


import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.List;

//THIS IS A SIDE PROJECT BY BAXTER, IF YOU ARE READING THIS, DO NOT IMPLEMENT THIS CLASS ANYWHERE!!!

public class StereoDistanceHelper  {

    Mat rightCameraImage;
    Mat leftCameraImage;
    List<MatOfPoint> rightContours;
    List<MatOfPoint> leftContours;
    final double FOV = 55;
    final double CAMERA_DISTANCE = 0.215;

    //Constructor
    public StereoDistanceHelper(Mat rightCameraImage, Mat leftCameraImage, List<MatOfPoint> rightContours, List<MatOfPoint> leftContours){
        this.rightCameraImage = rightCameraImage;
        this.leftCameraImage = leftCameraImage;
        this.rightContours = rightContours;
        this.leftContours = leftContours;
    }

    //Find the points in the image
    public int[] getPoints(List<MatOfPoint> contours){
        double minimum = 0.0;
        double maximum = 50.0;

        int[] points = new int[20];

        int i = 0;

        for (MatOfPoint contour : contours){
            MatOfPoint2f areaPoints = new MatOfPoint2f(contour.toArray());
            RotatedRect boundingRect = Imgproc.minAreaRect(areaPoints);
            //Do some reading on above functions
            //define area
            double rectangleArea = boundingRect.size.area();
            int rectCount = 0;
            if(rectangleArea > minimum && rectangleArea < maximum){//Threshold for what size to consider correct target
                rectCount++;
                Point[] rotated_rect_points = new Point[4];
                final Scalar BLUE = new Scalar(0, 0, 255);
                boundingRect.points(rotated_rect_points); //Populate points
                Rect currRect = Imgproc.boundingRect(new MatOfPoint(rotated_rect_points));//Creates rectangle object that we can draw
                //Another good function to look up and read about
                Imgproc.rectangle(rightCameraImage,currRect,BLUE,2);
                //Passes the current image, current Rectangle, color to draw, and line thickness
                //Imgproc.putText(rightCameraImage, "Rectangle: " + rectCount, currRect.br(), 1, 1, BLUE);
                points[i] = currRect.x + (currRect.width/2);
            }
            //Something to think about
            //Store rectangles in array and create method to return array when called so you can do math with the rectangles
            i++;
        }
        int[] returnPoints = new int[points.length];

        for(int index = 0; index < returnPoints.length; index++){
            returnPoints[index] = points[index];
        }

        return returnPoints;
    }

    //Find the distance using the instance variables
    public double[] process(){

        int[] rightPoints = getPoints(rightContours);
        int[] leftPoints = getPoints(leftContours);

        double[] distances = new double[rightPoints.length];

        for(int i = 0; i < rightPoints.length; i++){
            distances[i] = calculateDistance(rightPoints[i],leftPoints[i]);
        }

        return distances;
    }

    //Return the distance of the object
    public double calculateDistance(int x1, int x2){
        return (rightCameraImage.cols()*CAMERA_DISTANCE)/(2 * Math.tan(FOV/2)*(x1-x2));
    }

    //Pair two coordinates to one another for detection
    public int[][] pairProximity(int[][] points){
        for(int i = 0; i < points[0].length; i++){
            int[] point = {points[0][i], points[1][i]};



            for(int contender = 0; contender < points[0].length; contender++){
                int[] contenderPoint = {points[2][contender], points[3][contender]};

                int[][] shortestDistance = new int[4][1];

                int xDiff = Math.abs(contenderPoint[0] - point[0]);
                int yDiff = Math.abs(contenderPoint[1] - point[1]);

                double distance = Math.sqrt((xDiff*xDiff) + (yDiff*yDiff));


            }
        }

        return null;
    }
}
