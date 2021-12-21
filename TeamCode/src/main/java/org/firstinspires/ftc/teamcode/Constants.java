package org.firstinspires.ftc.teamcode;

public final class Constants {
    public final class MotorConstants {
        public final class RevHDHexMotor {
            public static final int ticks_per_revolution = 72;
        }
    }

    public final class LiftConstants {
        public static final int inverse_motor_gear = 4 * 4 * 3;
        public static final int ticks_per_motor_revolution = 10 * inverse_motor_gear;
        public static final double gear = 15f / 10f;
        public static final int ticks_per_revolution = (int)(ticks_per_motor_revolution / gear);
        public static final double gear_radios = 0.0205;
        public static final double distance_per_revolution = 2 * Math.PI * gear_radios;
        public static final double distance_per_tick = distance_per_revolution / ticks_per_revolution;
    }
}
