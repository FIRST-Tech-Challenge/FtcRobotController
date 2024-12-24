package org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;

import org.firstinspires.ftc.teamcode.utils.PID.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.utils.geometry.BTTranslation2d;

public static class ChassisConstants {
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
        }

    public static final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(FLW, FRW, BLW, BRW);
    public static final double robotThetaVelocityMax = 180; //degree per sec
    public static final double robotThetaAccMax = 180; //degree per sec^2

    @Config
    public static class RotationPID{
        public static double rkp = 0.0;
        public static double rki = 0.0;
        public static double rkd = 0.0;
        public static double rks = 0;
        public static double degrees = 0;
        public static double tolerance = 0;
        public static double rotIzone = 0;

    @Config
    public static class PIDConstants {
        public static double Xkp = 0;
        public static double Xki = 0;
        public static double Xkd = 0;
        public static double Ykp = 0;
        public static double Yki = 0;
        public static double Ykd = 0;

        }
    }
}