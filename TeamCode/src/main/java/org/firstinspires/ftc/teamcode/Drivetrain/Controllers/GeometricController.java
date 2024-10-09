package org.firstinspires.ftc.teamcode.Drivetrain.Controllers;

import com.acmerobotics.dashboard.config.Config;

import org.ejml.simple.SimpleMatrix;

import java.util.LinkedHashSet;

@Config
public class GeometricController {
    public static double radiusXY = 10;
    public static double radiusTheta = 20;
    public double[][] wayPoints;
    public int lastLookahead = 0;
    public GeometricController(double[][] wp){
        wayPoints = wp;
    }
    double solveParameter(double x, double y, int i, double radius){
        double a = Math.pow((wayPoints[i+1][0] - wayPoints[i][0]), 2) + Math.pow((wayPoints[i+1][1] - wayPoints[i][1]), 2);
        double b = 2*((wayPoints[i][0]-x)*(wayPoints[i+1][0]-wayPoints[i][0]) + (wayPoints[i][1]-y)*(wayPoints[i+1][1]-wayPoints[i][1]));
        double c = Math.pow(wayPoints[i][0],2)-2*x*wayPoints[i][0]+Math.pow(x, 2)+Math.pow(wayPoints[i][1],2)-2*x*wayPoints[i][1]+Math.pow(y, 2)+Math.pow(radius, 2);
        double discriminant = Math.pow(b, 2) - 4 * a * c;
        if (discriminant>=0) {
            double rootOne = (-b + Math.sqrt(discriminant)) / (2 * a);
            return (rootOne >= 0 && rootOne <= 1) ? rootOne : (-b - Math.sqrt(discriminant)) / (2 * a);
        }
        return -1;
    }
    public double calculate(double x, double y){
        for (int i=lastLookahead; i< wayPoints.length-1; i++){
            double tI = solveParameter(x, y, i, radiusXY);
            double pX = wayPoints[i][0]+(wayPoints[i+1][0]-wayPoints[i][0])*tI;
            double pY = wayPoints[i][1]+(wayPoints[i+1][1]-wayPoints[i][1])*tI;
        }
        //work so far
    }
}
