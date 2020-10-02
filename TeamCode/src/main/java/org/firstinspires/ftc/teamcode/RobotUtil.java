package org.firstinspires.ftc.teamcode;

public class RobotUtil {

    public static double scaleVal (double input, double minInputVal, double maxInputVal, double minOutputVal, double maxOutputVal) {
        if (input > maxInputVal) input = maxInputVal;
        double inputRange = Math.abs(maxInputVal - minInputVal);
        double outputRange = Math.abs(maxOutputVal - minOutputVal);
        double scaleFactor = input/inputRange;
        return outputRange * scaleFactor + minOutputVal;
    }
}
