package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Waypoint;

public class AutonomousAwareness {
    /** Width of the track */
    static final double TRACK_WIDTH = 13.7;

    /** Amount of ticks per one inch */
    static final double TICKS_TO_INCHES = 15.3;

    static final double CENTER_WHEEL_OFFSET = 2.4;

    private static MotorEx encoderLeft, encoderRight, encoderBack;
    static HolonomicOdometry holOdom;
    
    static OdometrySubsystem odometry;

    public static Waypoint[] currentTask;

    public enum StartingPosition {
        RED_LEFT, 
        RED_RIGHT, 
        BLUE_LEFT, 
        BLUE_RIGHT
    }

    public void initOdometry() {
        encoderLeft = new MotorEx(hardwareMap, "encoderLeft");
        encoderRight = new MotorEx(hardwareMap, "encoderRight");
        encoderBack = new MotorEx(hardwareMap, "encoderBack");

        encoderLeft.setDistancePerPulse(TICKS_TO_INCHES);
        encoderRight.setDistancePerPulse(TICKS_TO_INCHES);
        encoderBack.setDistancePerPulse(TICKS_TO_INCHES);

        holOdom = new HolonomicOdometry(
            encoderLeft::getDistance,
            encoderRight::getDistance,
            encoderBack::getDistance,
            TRACK_WIDTH, CENTER_WHEEL_OFFSET
        );

        odometry = new OdometrySubsystem(holOdom);
    }

    /** 
     * Constructor for the AutonomousAwareness class
     * @param startingPosition the starting position of the robot
     * @param useDistanceSensor whether or not to use the distance sensor for more accurate positioning
     */
    public AutonomousAwareness(StartingPosition startingPosition, boolean useDistanceSensor) {
        initOdometry();
    }
    /**
     * Constructor for the AutonomousAwareness class
     * @param startingPosition the starting position of the robot
     */
    /*
    public AutonomousAwareness(StartingPosition startingPosition) {
        AutonomousAwareness(startingPosition, false);
    }
    */
}
