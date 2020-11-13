package org.firstinspires.ftc.teamcode.MISC;

public class CONSTANTS {
    // PID Constants
    public final static double[] dPid_ = new double[]{2.832556, 0.0000098546, 0.00008}; // distance pid values
    public final static double[] aPid_ = new double[]{0.000000504, 0.00000000002998311, 0.000000000000702983749}; // acceleration pid values
    public final static double[] vPid_ = new double[]{.0000274, 0.0000000004643, 0}; // velocity pid values
    public final static double[] imuPid_ = new double[]{0.000001246, 0.00000000000004, 0.000000000000000000}; // imu based pid values

    public final static double[] fusedDPid_ = new double[]{0, 0, 0};

    // PID Drive Parameters
    public final static double target_acceleration = 0.0000124;
    public final static double target_velocity     = .114;

    // Robot Parameters
    public final static double wheel_radius_meters_MECANUM = 0.0508;
    public final static double x02 = .18;
    public final static double encoder_count_per_rev_NV = 1120;
    public final static double encoder_count_per_rev_REV = 1120;
}
