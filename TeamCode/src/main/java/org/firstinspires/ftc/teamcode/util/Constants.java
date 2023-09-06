package org.firstinspires.ftc.teamcode.util;

public final class Constants<FORWARD_VALUES> {
    //math constants to speed up calculations
    public static final double SQRT2=Math.sqrt(2);
    public static final double SQRT2_OVER2 = SQRT2/2;
    public static final double PI_OVER4=Math.PI/4;
    public static final double PI_OVER2=Math.PI/2;
    public static final double INCHES_PER_METER=39.3701;
    public static final double PI_OVER_180=Math.PI/180;


    // ARM release and set position
    public static final double ARM_RELEASE_POS = 0.85;
    public static final double ARM_SET_POS = 0.2;

    // motor constants based on physical properties of the robot
    public static final double COUNTS_PER_MOTOR_REV = 1120;   // 1120 per revolution
    public static final double DRIVE_GEAR_REDUCTION = 0.75; //   3/4
    public static final double WHEEL_DIAMETER_INCHES = 96/25.4;     // For figuring circumference
    public static final double COUNTS_PER_INCH_FORWARD = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double ROBOT_DIAMETER_IN = 13;
    public static final double COUNTS_PER_ROTATE = (ROBOT_DIAMETER_IN * Math.PI)*COUNTS_PER_INCH_FORWARD;
    public static final int [] FORWARD_VALUES  = new int[]{ 1, 1, 1, 1,1};
    public static final int [] REVERSE_VALUES = new int[]{-1, -1, -1, -1,-1};
    public static final int [] LATERAL_LEFT_VALUES = new int[]{1,-1,1,-1,1};
    public static final int [] LATERAL_RIGHT_VALUES = new int[]{-1,1,-1,1,1};
    public static final int [] ROTATE_VALUES = new int[]{1,1,-1,-1};

    public static final double SPEED_FACTOR =1.4;
    public static final double ROTATION_RATE =0.75;

    // speed settings
    public static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    public static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    public static final double     HEADING_THRESHOLD       = 1 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value
    public static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    //maybe only use one of these.
    public static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable


    public final static String VuforiaKey = "Ac6tBZr/////AAABmd2+0ZS1DUaxvjeLOQXt6BocTj8MS8ZdGc3iaWgJcb4x+GTRiMydjRed7kvoAvq0x21glktV2ekv6Nq8WLNelf5Chl5vN4X9QjUKYvH1fgh72q2cY2w5lMO5tmoOAbyNlN4hSM+RdaWXC7MpY95EVbwz584eP2KUQ97DMCFYqGj6zaVTap2FQ/U2rK7XDNp+s0mdm1+2dvJh6bw0Xpp/DjkUG7RB3uLZe0niObsnONPJg29RCf2eOVY/NP7qjXZamhGLjR1Cpj+U2HGh5DIqCauT/lvn/PDfa+H8ErXG0grgeSqQUHGYlsnYiYrp7Q70RKeebAeOsMVVj6zNhjI6dGE06u3JZgT6aF5EMxnJyc2X";
}
