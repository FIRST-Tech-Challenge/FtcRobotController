package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Controllables.Location;

/**
 * @author Zack Horton
 * @version 1.0
 * @since 1.0
 */
public class MathFunctions {

    /**
     * Method which takes in the current angle you are facing in radians and makes sure it is between -PI and PI
     * @param currentAngle Your supplied heading between -2PI and 2PI
     * @return The wrapped heading between -PI and PI
     */
    public static double angleWrap(double currentAngle){
        if(currentAngle < -Math.PI)
        {
            return (Math.PI * 2) + currentAngle;
        }
        else if (currentAngle > Math.PI)
        {
            return (-Math.PI * 2) + currentAngle;
        }
        else
        {
            return currentAngle;
        }
    }

    /**
     * Method to find the shortest angle between your current angle and your target angle
     * @param currentAngle Supplied current angle of bot
     * @param targetAngle Target angle
     * @return The shortest difference between current angle and target angle
     */
    public static double smallestAngleDiff(double currentAngle, double targetAngle){
        targetAngle -= currentAngle;
        return angleWrap(targetAngle) + currentAngle;
    }

    public static double distance(double x1, double y1, double x2, double y2){
        return Math.hypot((x2-x1),(y2-y1));
    }

    public static double distance(Location loc1, Location loc2){
        return Math.hypot((loc2.xPos - loc1.xPos), (loc2.yPos - loc1.yPos));
    }

    public static double distance(Location loc, double x, double y){
        return Math.hypot((x - loc.xPos), (y - loc.yPos));
    }

    public static double clip(double val, double high, double low) {
        if (val > high)
        {
            val = high;
        }
        if (val < low)
        {
            val = low;
        }
        return val;
    }

    public static double quadratic1(double a, double b, double c){
        if (Math.sqrt(Math.pow(b, 2) - (4 * a * c)) < 0) //Checks to make sure there are solutions
        {
            return 0;
        }
        return (-b + Math.sqrt(Math.pow(b, 2) - (4 * a * c))) / (2 * a);
    }

    public static double quadratic2(double a, double b, double c){
        if (Math.sqrt(Math.pow(b, 2) - (4 * a * c)) < 0) //Checks to make sure there are solutions
        {
            return 0;
        }
        return (-b - Math.sqrt(Math.pow(b, 2) - (4 * a * c))) / (2 * a);
    }

    public static boolean quadraticChecker(double a, double b, double c){
        return Math.sqrt(Math.pow(b, 2) - (4 * a * c)) >= 0;
    }

    public static Location lineCircleIntersection(Location start, Location end, double x, double y, double radius)
    {
        double m = (end.yPos - start.yPos) / (end.xPos - start.xPos);
        double b = start.yPos - (m * start.xPos);

        double quadA = (1 + Math.pow(m, 2));
        double quadB = ((2 * m * b) + (2 * m * y) + (2 * x));
        double quadC = (Math.pow(b, 2) + Math.pow(y, 2) - Math.pow(radius, 2) - (2 * b * y));

        double xSolution1 = 0;
        double xSolution2 = 0;

        //Initialize the best solution to the equations to zero (best being closest to the end point)
        double xSolution = 0;

        if (MathFunctions.quadraticChecker(quadA, quadB, quadC)) //Checks to make sure that the circle intersects with the line
        {
            //Sets the two possible solutions to the solution of the quadratic formula to the x solutions
            xSolution1 = MathFunctions.quadratic1(quadA, quadB, quadC);
            xSolution2 = MathFunctions.quadratic2(quadA, quadB, quadC);

            //Chooses the best solution to be the actual solution. Best being closest to the end point
            if (start.xPos < end.xPos) {
                xSolution = Math.max(xSolution1, xSolution2);
            } else {
                xSolution = Math.min(xSolution1, xSolution2);
            }
        } else        //If the circle doesn't intersect with the line, go straight towards the line perpendicularly to it
        {
            xSolution = end.xPos;
        }

        double ySolution = (m * xSolution) + b;

        return new Location(xSolution, ySolution, 0);

    }

}
