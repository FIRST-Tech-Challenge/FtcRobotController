package org.firstinspires.ftc.teamcode;

public final class Constants {

    public static final class DriveConstants {
        public static final double kInchToMm = 25.4;
        public static final int kFrontLeftMotor = 1;
        public static final int kFrontRightMotor = 2;
        public static final int kBackLeftMotor = 3;
        public static final int kBackRightMotor = 4;
        public static final double kCmParTour = Math.PI*10; //Circonference en cm
        public static final double kTickParTour = 1440;
        public static final double kCmParTick = kCmParTour/kTickParTour;

    }
}
