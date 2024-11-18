package org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil;

import com.acmerobotics.roadrunner.Vector2d;

public class MathUtil {
    public static Vector2d rotateVec(Vector2d vec, double angle) {
        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);
        return new Vector2d(

                vec.component1() * sinA + vec.component2() * cosA,
                vec.component1() * cosA - vec.component2() * sinA
        );
    }

    public static double aplayDeadzone(double input, double deadzone) {
        if (Math.abs(input) < deadzone) {
            return 0;
        }
        return input;
    }
    public static double convertTicksToDistance(double CPR,double diameter,double ticks) {
        double circumference = Math.PI * diameter;
        double revolutions = ticks/CPR;
        double distance = circumference * revolutions;
        return distance;
    }
    public static double convertDistanceToTicks(double CPR,double diameter,double distance) {
        double circumference = Math.PI * diameter;
        double revolutions = distance/circumference;
        double ticks = revolutions*CPR;
        return ticks;
    }
    public static double convertTicksToDegries(double CPR,double ticks){
        double revolutions = ticks/CPR;
        double angle = revolutions * 360;
        double angleNormalized = angle % 360;
        return angleNormalized;
    }
    public static double convertDegriesToTicks(double CPR, double angle){
        double revolutions = angle/360;
        double ticks = revolutions *CPR;
        return ticks;

    }
    public static double voltageToDegrees(double voltage){
        return voltage/ 3.3*360;
    }

    public static boolean inTolerance(double dp,double cp,double tolerance){
        return Math.abs(dp - cp) > tolerance;
    }
}

