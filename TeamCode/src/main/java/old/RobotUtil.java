package old;

public class RobotUtil {

    public static double scaleVal (double input, double minInputVal, double maxInputVal, double minOutputVal, double maxOutputVal) {
        if (input > maxInputVal) input = maxInputVal;
        if (input < minInputVal) input = minInputVal;
        double inputRange = Math.abs(maxInputVal - minInputVal); //abs value should be redundant (max > min)
        double outputRange = Math.abs(maxOutputVal - minOutputVal);
        if (inputRange == 0) return input; //added
        return (outputRange/inputRange) * (input - minInputVal) + minOutputVal;
    }

    public static double mapToAbs(double input, double maxOutput, double maxInput) {
        return  Math.abs(input) > maxInput ? maxOutput : //If out of range
                Math.abs((maxOutput / maxInput) * input);
    }

    public static double mapToLineThroughOrigin(double input, double maxOutput, double maxInput) {
        return  input > maxInput ? maxOutput :
                input < -maxInput ? -maxOutput :
                        (maxOutput / maxInput) * input;
    }

    //public static double scaleVal(double input, double minInputVal, double maxInputVal, double minOutputVal, double maxOutputVal) { return input > maxInputVal ? maxOutputVal : (input < minInputVal ? minOutputVal : (((maxOutputVal - minOutputVal) / (maxInputVal - minInputVal)) * (input - minInputVal) + minOutputVal)); }
}
