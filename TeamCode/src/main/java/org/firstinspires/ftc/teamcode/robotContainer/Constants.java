package org.firstinspires.ftc.teamcode.robotContainer;

public final class Constants
{
    // enum to specify opMode type
    public enum OpModeType {
        TELEOP,
        BLUE_RIGHT_AUTO,
        BLUE_LEFT_AUTO,
        RED_RIGHT_AUTO,
        RED_LEFT_AUTO
    }

    public static int LOW = 1200;
    public static int MEDIUM = 1500;
    public static int HIGH = 2400;
    public static int LOWLOW = 900;

    public static double DEFAULT_INTAKE_SPEED = 1.0;

    public enum OperatorMode {
        SINGLE_OPERATOR_MODE,
        DOUBLE_OPERATOR_MODE
    }
}
