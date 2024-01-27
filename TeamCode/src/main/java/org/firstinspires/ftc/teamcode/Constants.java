package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

public class Constants {
    public static final double l1ff = 1;// com distant from axis first arm METERS
    public static final double l2ff = 0.70;// com distant from axis second arm METERS
    public static final double first_arm_weight = 1.7; // first arm weight KG
    public static final double second_arm_weight = 1.3; // second arm weight with gripper KG
    public static final double g = 9.806;
    public static final double hex_stall_current = 181;
    public static final double resistance = 12 / hex_stall_current; //volt

    public static final double gear_ratio = 80;
    public static final double hex_stall_torque = 0.182; //N * meter

    public static final double neo_Kt = hex_stall_torque / hex_stall_current;

    public static final double motorMaxVolt = 12;
    public static final double vMax = -1;//placeholder value
    public static final double vMin = -1;//placeholder value


    public static class ChassisConstants {
        public static final double odometryWheelRadius = 0.0176; //meters
        public static final int tickPerRevolution = 8192;
        public static final double TimeToAprilTagCheck = 1;
        public static final double TRACKWIDTH = 1;
        public static final double WHEEL_OFFSET = 1;
        public static final double TICKS_TO_CM = 38.862;
        @Config
        public static class PIDConstants {
            public static double kp = 0;
            public static double ki = 0;
            public static double kd = 0;
            public static double kff = 0;
        }
    }





    final public static double cameraAngle = 0;
    public static class  Climb{
        public static final double climb_max_speed = 0;// todo: this is not calibrated
        public static final double climb_max_accel = 0;// todo: this is not calibrated
        public static final double kp = 0;// todo: this is not calibrated
        public static final double ki = 0;// todo: this is not calibrated
        public static final double kd = 0;// todo: this is not calibrated
        public static final double kf = 0;// todo: this is not calibrated
        public static final int max_ticks = 0;// todo: this is not calibrated

//    public enum ArmStates {
//        base(Arm.armBasePosition,false,ArmPlacingStates.base), // this doesn't need any boolean has a command on his own
//
//        public Translation2d desiredPoint;
//        //false is positive x true is negative
//        public boolean direction;
//        public ArmPlacingStates placingHeight;
//
//        private ArmStates(Translation2d point) {
//            this.desiredPoint = point;
//
//        }
//    }
//
}
}
