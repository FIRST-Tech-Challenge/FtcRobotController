package org.firstinspires.ftc.teamcode.Drivetrain.Controllers;

import com.acmerobotics.dashboard.config.Config;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashSet;
import org.firstinspires.ftc.teamcode.Drivetrain.Geometry.Path;

@Config
public class GeometricController {
    public static double lookAheadXY = 10;
    public static double lookAheadTheta = 20;
    public boolean useStaticHeading = false;
    int lastLookahead = 0;
    public GeometricController(){
    }
    double[] calcCircleLineIntersection(double xPos, double yPos, int i, double radius, double[][] wayPoints){
        double a = Math.pow((wayPoints[i+1][0] - wayPoints[i][0]), 2) + Math.pow((wayPoints[i+1][1] - wayPoints[i][1]), 2);
        double b = 2*((wayPoints[i][0]-xPos)*(wayPoints[i+1][0]-wayPoints[i][0]) + (wayPoints[i][1]-yPos)*(wayPoints[i+1][1]-wayPoints[i][1]));
        double c = Math.pow(wayPoints[i][0],2)-2*xPos*wayPoints[i][0]+Math.pow(xPos, 2)+Math.pow(wayPoints[i][1],2)-2*xPos*wayPoints[i][1]+Math.pow(yPos, 2)+Math.pow(radius, 2);
        double discriminant = Math.pow(b, 2) - 4 * a * c;
        double root = 0;
        if (discriminant>=0) {
            double rootOne = (-b + Math.sqrt(discriminant)) / (2 * a);
            double rootTwo = (-b - Math.sqrt(discriminant)) / (2 * a);
            if (rootOne>=0 && rootOne<=1 && rootTwo>=0 && rootTwo<=1){
                root = Math.max(rootOne, rootTwo);
            } else if (rootOne>=0 && rootOne<=1){
                root = rootOne;
            } else if (rootTwo>=0 && rootTwo<=1){
                root = rootTwo;
            }
            double pX = wayPoints[i][0]+(wayPoints[i+1][0]-wayPoints[i][0])*root;
            double pY = wayPoints[i][1]+(wayPoints[i+1][1]-wayPoints[i][1])*root;
            return new double[]{pX, pY};
        }
        return new double[]{-99999, -99999};
    }
    public SimpleMatrix calculate(double x, double y, Path path){
        double[][] wayPoints = path.waypoints;
        LinkedHashSet<double[]> potentialPoints = new LinkedHashSet<>();
        LinkedHashSet<double[]> thetaPoints = new LinkedHashSet<>();
        for (int i=lastLookahead; i<wayPoints.length-1; i++){
            double[] intersection = calcCircleLineIntersection(x,y,i,lookAheadXY, wayPoints);
            if (!Arrays.equals(intersection, new double[]{-99999, -99999})) {
                potentialPoints.add(intersection);
            }
        }
        for (int i=lastLookahead; i<wayPoints.length-1; i++){
            double[] intersection = calcCircleLineIntersection(x,y,i,lookAheadTheta, wayPoints);
            if (!Arrays.equals(intersection, new double[]{-99999, -99999})) {
                thetaPoints.add(intersection);
            }
        }
        ArrayList<double[]> thetaArray = new ArrayList<>(thetaPoints);
        ArrayList<double[]> posArray = new ArrayList<>(potentialPoints);
        double[] furthestPoint = thetaArray.get(thetaArray.size()-1);
        double desiredTheta;
        if (path.useStaticHeading) {
            desiredTheta = path.finalHeading;
        } else {
            desiredTheta = Math.atan2((thetaArray.get(thetaArray.size()-1)[1]-y), (thetaArray.get(thetaArray.size()-1)[0]-x));
            if (path.reverse) {
                if (Math.signum(desiredTheta)==-1){
                    desiredTheta += Math.PI;
                } else if (Math.signum(desiredTheta)==0){
                    desiredTheta -= Math.PI;
                }
            }

        }
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
