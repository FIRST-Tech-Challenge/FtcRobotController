package org.firstinspires.ftc.teamcode.Drivetrain.Controllers;

import com.acmerobotics.dashboard.config.Config;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashSet;

@Config
public class GeometricController {
    public static double lookAheadXY = 10;
    public static double lookAheadTheta = 20;
    int lastLookahead = 0;
    public GeometricController(){
    }
    double[] calcCircleLineIntersection(double xPos, double yPos, int i, double radius, double[][] wayPoints){
        double a = Math.pow((wayPoints[i+1][0] - wayPoints[i][0]), 2) + Math.pow((wayPoints[i+1][1] - wayPoints[i][1]), 2);
        double b = 2*((wayPoints[i][0]-xPos)*(wayPoints[i+1][0]-wayPoints[i][0]) + (wayPoints[i][1]-yPos)*(wayPoints[i+1][1]-wayPoints[i][1]));
        double c = Math.pow(wayPoints[i][0],2)-2*xPos*wayPoints[i][0]+Math.pow(xPos, 2)+Math.pow(wayPoints[i][1],2)-2*xPos*wayPoints[i][1]+Math.pow(yPos, 2)+Math.pow(radius, 2);
        double discriminant = Math.pow(b, 2) - 4 * a * c;
        if (discriminant>=0) {
            double rootOne = (-b + Math.sqrt(discriminant)) / (2 * a);
            double root = (rootOne >= 0 && rootOne <= 1) ? rootOne : (-b - Math.sqrt(discriminant)) / (2 * a);
            double pX = wayPoints[i][0]+(wayPoints[i+1][0]-wayPoints[i][0])*root;
            double pY = wayPoints[i][1]+(wayPoints[i+1][1]-wayPoints[i][1])*root;
            return new double[]{pX, pY};
        }
        return new double[]{-99999, -99999};
    }
    public SimpleMatrix calculate(double x, double y, double[][] wayPoints){
        LinkedHashSet<double[]> potentialPoints = new LinkedHashSet<>();
        LinkedHashSet<double[]> thetaPoints = new LinkedHashSet<>();
        for (int i=lastLookahead; i<wayPoints.length-1; i++){
            potentialPoints.add(calcCircleLineIntersection(x,y,i,lookAheadXY, wayPoints));
        }
        for (int i=lastLookahead; i<wayPoints.length-1; i++){
            thetaPoints.add(calcCircleLineIntersection(x,y,i,lookAheadTheta, wayPoints));
        }
        ArrayList<double[]> thetaArray = new ArrayList<>(thetaPoints);
        ArrayList<double[]> posArray = new ArrayList<>(potentialPoints);
        double[] furthestPoint = thetaArray.get(thetaArray.size()-1);
        double desiredTheta = Math.atan2((furthestPoint[1]-y), (furthestPoint[0]-x));
        SimpleMatrix pose = new SimpleMatrix(
            new double[]{
                    posArray.get(thetaArray.size()-1)[0],
                    posArray.get(thetaArray.size()-1)[1],
                    desiredTheta
            }
        );
        return pose;
    }
    public void resetLookAhead(){
        lastLookahead = 0;
    }
}
