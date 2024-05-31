package org.firstinspires.ftc.teamcode.constants;

import org.firstinspires.ftc.teamcode.org.rustlib.control.PIDController;

public class DriveConstants {
    public static double maxEndpointErr = 0.5;
    public static double defaultFollowRadius = 8;
    public static double trackEndpointHeadingMaxDistance = 12.0;
    public static double calculateTargetHeadingMinDistance = 15.0;
    public static double maxFinalVelocityInPerSec = 1.0;
    public static PIDController.PIDGains driveGains = new PIDController.PIDGains(0.1, 0.002, 0.0001);
    public static PIDController.PIDGains rotGains = new PIDController.PIDGains(1.0, 0, 0);

    public static class Odometry {
        public static double trackWidth = 13.658011373578302712160979877515;
        public static double verticalDistance = 7.5035;
        public static double inPerTick = 0.002968431495;
    }
}
