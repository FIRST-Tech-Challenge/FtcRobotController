package org.firstinspires.ftc.teamcode.Drivetrain.Controllers;

import com.acmerobotics.dashboard.config.Config;

import org.ejml.simple.SimpleMatrix;

import java.util.LinkedHashSet;

@Config
public class GeometricController {
    public static double radiusXY = 10;
    public static double radiusTheta = 20;
    public double[][] wayPoints;
    public GeometricController(){
    }
    double calcCircleLineIntersection(double xPos, double yPos, int i, double radius){
        double a = Math.pow((wayPoints[i+1][0] - wayPoints[i][0]), 2) + Math.pow((wayPoints[i+1][1] - wayPoints[i][1]), 2);
        double b = 2*((wayPoints[i][0]-xPos)*(wayPoints[i+1][0]-wayPoints[i][0]) + (wayPoints[i][1]-yPos)*(wayPoints[i+1][1]-wayPoints[i][1]));
        double c = Math.pow(wayPoints[i][0],2)-2*xPos*wayPoints[i][0]+Math.pow(xPos, 2)+Math.pow(wayPoints[i][1],2)-2*xPos*wayPoints[i][1]+Math.pow(yPos, 2)+Math.pow(radius, 2);
        double discriminant = Math.pow(b, 2) - 4 * a * c;
        if (discriminant>=0) {
            double rootOne = (-b + Math.sqrt(discriminant)) / (2 * a);
            double root = (rootOne >= 0 && rootOne <= 1) ? rootOne : (-b - Math.sqrt(discriminant)) / (2 * a);
            double pX = wayPoints[i][0]+(wayPoints[i+1][0]-wayPoints[i][0])*root;
            double pY = wayPoints[i][1]+(wayPoints[i+1][1]-wayPoints[i][1])*root;
        }
        return -1;
    }
    public double calculate(double x, double y, double[][] wayPoints){
        int lastLookahead = 0;
        LinkedHashSet<double[]> potentialPoints = new LinkedHashSet<>();
        for (int i=lastLookahead; i< wayPoints.length-1; i++){

        }
        //work so far
    }
}
