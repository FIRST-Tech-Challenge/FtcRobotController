package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;

public class Variables {

    /**
     * This class is for organization of all variables to be used across the codebase. The
     * procedure for naming (with the exception of constants) is the "type" of variable
     * (motor, servo, etc.) then following what it is used for (see below)
     */

    //motors
    public static DcMotor motorFrontLeft;
    public static DcMotor motorBackLeft;
    public static DcMotor motorFrontRight;
    public static DcMotor motorBackRight;

    //servos


    //sensors
    public static Gyroscope imu;

    //other (static constants for computation)
    public static final double CLICKS_PER_ROTATION = 537.6;
    public static final double DISTANCE_PER_ROTATION = 0.3015928947; //In meters
    public static final double DISTANCE_PER_CLICK = (DISTANCE_PER_ROTATION/CLICKS_PER_ROTATION);
}
