package org.firstinspires.ftc.teamcode.Functions.MV;

import org.firstinspires.ftc.teamcode.Functions.MV.MVVariables;

public class MVTurnTowardsPoint {


    /**
     * This method calculates the angle which is needed to be achieved so that the robot heads to a specific point.
     * @param destinationX : self explanatory
     * @param destinationY : self explanatory
     * @param currentPositionX : self explanatory
     * @param currentPositionY : self explanatory
     * @param currentAngle : self explanatory
     * @return : This returns an angle.
     */
    public static double AngleCalculator(int destinationX, int destinationY, int currentPositionX, int currentPositionY, double currentAngle)
    {

        MVVariables.Vector2 destination = new MVVariables.Vector2(destinationX, destinationY);
        MVVariables.Vector2 currentPosition = new MVVariables.Vector2(currentPositionX, currentPositionY);
        double distance = AutonomyDistanceCalculator(currentAngle, currentPosition, destination);

        double beta = Math.asin(Math.abs(currentPositionY - destinationY) / distance);

        return Math.toDegrees(beta) + 90 * (QuadrantPoint(currentPosition, destination)-1);
    }

    /**
     * This method does the same thing as the above one except the input comes from Variables.Vector2
     * @param destination : self explanatory
     * @param currentPosition : self explanatory
     * @param currentAngle : self explanatory
     * @return : This returns an angle.
     */
    public static double AngleCalculator(MVVariables.Vector2 destination, MVVariables.Vector2 currentPosition, double currentAngle)
    {
        double distance = AutonomyDistanceCalculator(currentAngle, currentPosition, destination);

        double beta = Math.asin(Math.abs(currentPosition.y - destination.y) / distance);

        return AngleCorrection((int)Math.toDegrees(beta) + 90 * (QuadrantPoint(currentPosition, destination) - 1));
        /*
        if(destination.x!=0&&destination.y!=0) {
             distance = AutonomieDistantaCalculare(currentAngle, currentPosition, destination);

             beta = Math.asin(Math.abs(currentPosition.y - destination.y) / distance);

            return Math.toDegrees(beta) + 90 * (CalcCadranPunct(currentPosition, destination) - 1);
        }
        else if(destination.x!=0){
            if(destination.x>0)
                return 0;
            if(destination.x<0)
                return 180;
        }
        else if(destination.y!=0){
            if(destination.y>0)
                return 90;
            if(destination.y<0)
                return 270;
        }
        return 0;
        */

    }

    /**
     * This method calculates the dial/quadrant (cadran) in which the point is located towards another point.
     * @param origin : self explanatory
     * @param destination : self explanatory
     * @return : This returns number of quadrant.
     */
    static int QuadrantPoint(MVVariables.Vector2 origin, MVVariables.Vector2 destination)
    {
        double deltaX = -origin.x + destination.x,
                deltaY = -origin.y + destination.y;
        if(deltaX >= 0 && deltaY >= 0)
        {
            return 1;
        }
        else if (deltaX <= 0 && deltaY >= 0)
        {
            return 2;
        }
        else if (deltaX <= 0 && deltaY <= 0)
        {
            return 3;
        }
        else if (deltaX >= 0 && deltaY <= 0)
        {
            return 4;
        }
        return 0;
    }

    /**
     * This method calculates the distance between the 2 vectors/arrays.
     * @param currentAngle : self explanatory
     * @param currentPosition : self explanatory
     * @param targetPosition : self explanatory
     * @return : This returns distance between the 2 vectors/arrays.
     */
    static double AutonomyDistanceCalculator(double currentAngle, MVVariables.Vector2 currentPosition, MVVariables.Vector2 targetPosition)
    {
        double x=currentAngle;
        double targetDistance = 0;
        double quadrant = NormalizeAngle(currentAngle);
        double y = 180 - x;


        double ax, bx, ay, by;
        ax = currentPosition.x;
        bx = targetPosition.x;
        ay = currentPosition.y;
        by = targetPosition.y;
        double hypotenuse2 = Math.abs(Math.sqrt((bx - ax) *(bx - ax) + (ay - by) * (ay - by)));

        return hypotenuse2;

    }

    /**
     * This method normalizes a given angle.
     * @param currentAngle
     * @return
     */
    static double NormalizeAngle(double currentAngle)
    {
        double newAngle;
        double quadrant = 1;
        newAngle = currentAngle;
        while (newAngle - 90 >= 0)
        {
            newAngle -= 90;
            quadrant++;
        }
        return quadrant;
    }

    /**
     * This method corrects the given angle. If it's above 360, decreases it by 360, and if it's below 0, adds 360 to it.
     * @param angle : (int) given angle
     * @return : This returns final corrected angle.
     */
    public static int AngleCorrection(int angle){
        int auxAngle=angle;
        while(auxAngle<0){
            auxAngle=auxAngle+360;
        }
        while(auxAngle>=360){
            auxAngle=auxAngle-360;
        }
        return auxAngle;
    }

}
