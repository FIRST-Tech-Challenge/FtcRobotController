package global;

public class Constants {
    public static final double MAX_OUTTAKE_SPEED = 200 * Math.PI; // rad/s

    public static final double OUTTAKE_ANGLE = 20 * Math.PI/180; // degrees -> radians
    public static final double GOAL_FROM_LEFT = 0.9; // meters
    public static final double GOAL_HEIGHT = 0.9; // meters
    public static final double SHOOTER_HEIGHT = 0.25; // meters
    public static final double SHOOTER_WHEEL_RADIUS = 0.05; // meters

    public static final double ROBOT_WIDTH = 30; // meters
    public static final double ROBOT_LENGTH = 33; // meters
    public static final double ROBOT_RADIUS = Math.sqrt(Math.pow(ROBOT_WIDTH/2, 2) + Math.pow(ROBOT_LENGTH/2, 2)); // meters
    public static final double CENTER_THETA = Math.PI - Math.atan2(ROBOT_LENGTH, ROBOT_WIDTH); // radians

}
