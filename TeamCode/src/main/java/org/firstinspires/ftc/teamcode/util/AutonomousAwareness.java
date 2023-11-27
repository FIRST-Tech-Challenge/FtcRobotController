package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.geometry.Pose2d;

public class AutonomousAwareness {
    /** Width of the track */
    static final double TRACK_WIDTH = 13.7;

    /** Amount of ticks per one inch */
    static final double TICKS_TO_INCHES = 15.3;

    static final double CENTER_WHEEL_OFFSET = 2.4;

    static MotorEx encoderLeft, encoderRight, encoderBack;
    static HolonomicOdometry holOdom;
    
    static OdometrySubsytem odometry;

    public AutonomousAwareness() {
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

        odometry = new OdometrySubsytem(holOdom);
    }
}
