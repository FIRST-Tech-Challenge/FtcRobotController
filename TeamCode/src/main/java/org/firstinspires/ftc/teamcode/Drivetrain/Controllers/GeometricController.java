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
    int lastLookaheadXY = 0;
    int lastLookaheadTheta = 0;
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
            } else {
                return new double[]{-99999, -99999};
            }
            // Pretty sure you need to handle the case where neither root is between 0 and 1
            // Ensure you use the -99999 return statement in that case
            
            double pX = wayPoints[i][0]+(wayPoints[i+1][0]-wayPoints[i][0])*root;
            double pY = wayPoints[i][1]+(wayPoints[i+1][1]-wayPoints[i][1])*root;
            return new double[]{pX, pY};
        }
        return new double[]{-99999, -99999};
    }

    // To be more consistent with PoseController, pass in SimpleMatrix Pose here,
    // then first two lines grab the x and y position from the pose.
    public SimpleMatrix calculate(SimpleMatrix posPose, Path path){
        double x = posPose.get(0,0);
        double y = posPose.get(1,0);
        // Assuming you read my Path class feedback, might want to change to path.getWaypoints() :-)
        double[][] wayPoints = path.getWaypoints();

        // rename to xyPoints to be consistent with thetaPoints!
        LinkedHashSet<double[]> potentialPoints = new LinkedHashSet<>();
        LinkedHashSet<double[]> thetaPoints = new LinkedHashSet<>();
        for (int i=lastLookaheadXY; i<wayPoints.length-1; i++){
            double[] intersection = calcCircleLineIntersection(x,y,i,lookAheadXY, wayPoints);
            if (!Arrays.equals(intersection, new double[]{-99999, -99999})) {
                potentialPoints.add(intersection);
            }
        }
        // You may want to have a 'lastLookaheadXY' AND a 'lastLookaheadTheta' as they may not be the same
        // Make sure to reset them both in the reset function
        for (int i=lastLookaheadTheta; i<wayPoints.length-1; i++){
            double[] intersection = calcCircleLineIntersection(x,y,i,lookAheadTheta, wayPoints);
            if (!Arrays.equals(intersection, new double[]{-99999, -99999})) {
                thetaPoints.add(intersection);
            }
        }

        // CONSIDER WHAT HAPPENS IF YOU ARE ON THE LAST SEGMENT OF THE PATH!! YOU MAY HAVE NO INTERSECTION
        // IF THE LOOKAHEAD CIRCLE IS LARGER THAN THE LINE SEGMENT. Check this for both the xy points and 
        // turn-to points. Please handle this edgecase!


        ArrayList<double[]> furthestIntersectionPointTheta = new ArrayList<>(thetaPoints);
        ArrayList<double[]> furthestIntersectionPointXY = new ArrayList<>(potentialPoints);
        
        // Rename this to furthestIntersectionPointTheta or something more descriptive.
        // Also do the same for furthestIntersectionPointXY!!!
        double[] furthestPoint = furthestIntersectionPointTheta.get(furthestIntersectionPointTheta.size()-1);
    
        double desiredTheta;

        if (path.useStaticHeading) {
            desiredTheta = path.finalHeading;
        } else {
            // You get furthestIntersectionPointTheta above for a reason. Use it to make this more readable!
            desiredTheta = Math.atan2((furthestIntersectionPointTheta.get(furthestIntersectionPointTheta.size()-1)[1]-y), (furthestIntersectionPointTheta.get(furthestIntersectionPointTheta.size()-1)[0]-x));
            if (path.reverse) {
                if (Math.signum(desiredTheta)==-1){
                    desiredTheta += Math.PI;
                } else if (Math.signum(desiredTheta)==0){
                    desiredTheta -= Math.PI;
                }
            }

        }

        // You are calling to thetaArray in the position spots!
        // See above: get the xy furthest point (furthestIntersectionPointXY)
        // and use that!
        SimpleMatrix pose = new SimpleMatrix(
            new double[]{
                    furthestIntersectionPointXY.get(furthestIntersectionPointTheta.size()-1)[0],
                    furthestIntersectionPointXY.get(furthestIntersectionPointTheta.size()-1)[1],
                    desiredTheta
            }
        );
        return pose;
    }
    public void resetLookAhead(){
        lastLookaheadXY = 0;
        lastLookaheadTheta = 0;
    }
}