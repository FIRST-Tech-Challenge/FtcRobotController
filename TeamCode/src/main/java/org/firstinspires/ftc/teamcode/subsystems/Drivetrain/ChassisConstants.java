package org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;

import org.firstinspires.ftc.teamcode.utils.PID.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.utils.geometry.BTTranslation2d;

public class ChassisConstants {
    public static final TrapezoidProfile.Constraints TrapezoidConstraints = new TrapezoidProfile.Constraints(1.63, 1.47);
    public static final double odometryWheelRadius = 0; //meters
    public static final int tickPerRevolution = 0;
    public static final double TimeToAprilTagCheck = 0;
    public static final double TRACKWIDTH = 0; //wheelbase, change value
    public static final double WHEEL_OFFSET = 0;
    public static final double TICKS_TO_CM = 0;
    public static final double RobotMaxVelFront = 0; // m/s
    public static final double RobotMaxVelSide = 0; // m/s
    public static final double RobotMaxAccFront = 0; // m/s^2
    public static final BTTranslation2d FRW = new BTTranslation2d();
    public static final BTTranslation2d BRW = new BTTranslation2d();
    public static final BTTranslation2d FLW = new BTTranslation2d();
    public static final BTTranslation2d BLW = new BTTranslation2d();

    @Config
    public static class FeedForwardConstants {
        public static double ks = 0.085; //values from kkbotz
        public static double kv = 1;
        public static double bl = 0;
        public static double br = 0;
        public static double fl = 0;
        public static double fr = 0.8;
    }

    public static final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(FLW, FRW, BLW, BRW);
    public static final double robotThetaVelocityMax = 180; //degree per sec
    public static final double robotThetaAccMax = 180; //degree per sec^2

    @Config
    public static class RotationPID {
        public static double rkp = 0.0145;
        public static double rki = 0.0006;
        public static double rkd = 0.00;
        public static double rks = 0;
        public static double degrees = -100;
        public static double tolerance = 1;
        public static double rotIzone = 5;
    }

    @Config
    public static class PIDConstants {
        public static double Xkp = 3;
        public static double Xki = 0.05;
        public static double Xkd = 0.5;
        public static double Ykp = 4.5;
        public static double Yki = 8;
        public static double Ykd = 0.28;
        public static double XiZone = 0.1;
        public static double YiZone = 0.06;
        public static double yMaxIntegral=1;
        public static double ytolerance=0.015;//1.5 cm
        public static double xtolerance=0.025;//2 cm


    }

}