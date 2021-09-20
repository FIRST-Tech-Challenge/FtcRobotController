package org.firstinspires.ftc.teamcode.skills;

public class Geometry {

    public static double getDistance(double X, double Y, double targetX, double targetY){
        double currentX = X;
        double currentY = Y;

        //determine the new heading to the target
        double distanceX = Math.abs(targetX - currentX);
        double distanceY = Math.abs(targetY - currentY);
        double chord = Math.sqrt(distanceX*distanceX + distanceY * distanceY);
        return chord;
    }

    public static double getRadius(double angleChange, double chord){
        double alpha = 90 - angleChange;
        double halfChord = chord / 2;
        double cosAlpha = Math.cos(Math.toRadians(alpha));
        double radius = 0;
        if (cosAlpha != 0){
            radius= halfChord / cosAlpha;
        }
        return radius;
    }

    public static double getAngle(double targetVector, double currentHead){
        boolean clockwise = targetVector > currentHead;
        double shortAngle = Math.abs(targetVector - currentHead);
        if (shortAngle > 180){
            shortAngle = 360 - shortAngle;
            clockwise = !clockwise;
        }
        if (clockwise){
            shortAngle = - shortAngle;
        }
        return shortAngle;
    }
}
