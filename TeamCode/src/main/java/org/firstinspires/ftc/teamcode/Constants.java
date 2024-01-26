package org.firstinspires.ftc.teamcode;

public class Constants  {
    public static final double l1 = 1;// com distant from axis first arm METERS
    public static final double l2 = 0.70;// com distant from axis second arm METERS
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
