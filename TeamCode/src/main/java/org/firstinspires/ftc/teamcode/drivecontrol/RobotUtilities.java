package org.firstinspires.ftc.teamcode.drivecontrol;

public class RobotUtilities {
    public static double scaleDouble(double input, double maxInputVal, double minInputVal, double maxOutputVal, double minOutputVal) {
        if (input > maxInputVal) {
            input = maxInputVal;
        }
        if (input < minInputVal) {
            input = minInputVal;
        }
        double inputRange = maxInputVal - minInputVal;
        double outputRange = maxOutputVal - minOutputVal;
        // Prevent divide by zero
        if (inputRange == 0) {
            return input;
        }
        return (outputRange/inputRange) * (input - minInputVal) + minOutputVal;
    }
}
