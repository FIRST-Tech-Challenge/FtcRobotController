package org.firstinspires.ftc.teamcode.utility;

public class RotationUtil {
    /**
     * Counterclockwise increasing. Shortest amount that a target position
     * on a circle must turn to reach destination. Defaults to positive values
     * when target and destination are directly opposed
     * @param robotAngle Controllable angle of target
     * @param destAngle  Desired angle of target
     * @return amount that robotAngle must change to reach targetAngle
     */
    public static double turnLeftOrRight(double robotAngle, double destAngle, double cycleSize){
        // convert to 0-360 range
        robotAngle = mod(robotAngle, cycleSize);
        destAngle = mod(destAngle, cycleSize);

        // When the angle difference > half, subtract greater angle by cycle
        // so further subtractions will show the difference from the other direction
        if (Math.abs(destAngle - robotAngle) >= cycleSize/2.0){
            if(destAngle > robotAngle) destAngle -= cycleSize;
            else robotAngle -= cycleSize;
        }

        return destAngle - robotAngle == -180? 180:destAngle - robotAngle;
    }

    public static double mod(double a, double b){
        if (a < 0){
            return b - (Math.abs(a) % b);
        }
        return a % b;
    }
}
