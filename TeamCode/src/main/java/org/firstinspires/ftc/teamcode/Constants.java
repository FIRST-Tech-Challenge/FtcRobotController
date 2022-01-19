package org.firstinspires.ftc.teamcode;

import java.util.function.DoubleFunction;
import java.util.function.IntFunction;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;

public final class Constants {
    public static final class MotorConstants {
        public static final class RevHDHexMotor {
            public static final int ticks_per_revolution = 28;
        }

        public static final class REVThroughBoreEncoder {
            public static final int ticks_per_revolution = 8192;
        }
    }

    public static final class DriveTrainConstants {
        public final static double ticks_per_revolution =
                MotorConstants.REVThroughBoreEncoder.ticks_per_revolution;

        public final static MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
                new Translation2d(0.28, 0.34),
                new Translation2d(0.28, -0.34),
                new Translation2d(-0.28, 0.34),
                new Translation2d(-0.28, -0.34)
        );

        public final static double WheelDiameter = 0.096;

        public final static DoubleFunction<Integer> m_to_ticks = (double m) -> (int)(m / WheelDiameter / Math.PI * ticks_per_revolution);
        public final static IntFunction<Double> ticks_to_m = (int ticks) -> ticks * WheelDiameter * Math.PI / ticks_per_revolution;

        public final static double kV = 0.55;
        public final static double kStatic = 0.114;
        public final static double kA = 0.056;
    }

    public static final class LiftConstants {
        public static final int inverse_motor_gear = 4 * 4 * 3;
        public static final int ticks_per_motor_revolution = 10 * inverse_motor_gear;
        public static final double gear = 15f / 10f;
        public static final int ticks_per_revolution = (int)(ticks_per_motor_revolution / gear);
        public static final double gear_radios = 0.0205;
        public static final double distance_per_revolution = 2 * Math.PI * gear_radios;
        public static final double distance_per_tick = distance_per_revolution / ticks_per_revolution;
    }

    public static final class ArmConstants {
        public static final int gear = 3 * 3 * 5;
        public static final int motorGear = MotorConstants.RevHDHexMotor.ticks_per_revolution;
    }
}
