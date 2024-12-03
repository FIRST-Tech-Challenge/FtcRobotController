package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

/*
 * Hardware class for goBilda robot (12"x15" chassis with 96mm/3.8" goBilda mecanum wheels)
 */
public class Hardware2025Bot
{
    //====== REV CONTROL/EXPANSION HUBS =====
    LynxModule controlHub;
    LynxModule expansionHub;
    public double controlHubV   = 0.0; // Voltage supply of the control hub
    public double expansionHubV = 0.0; // Voltage supply of the expansion hub

    //====== INERTIAL MEASUREMENT UNIT (IMU) =====
    protected BNO055IMU imu    = null;
    public double imuHeadingAngle = 0.0;
    public double    imuTiltAngle = 0.0;

    //====== GOBILDA PINPOINT ODOMETRY COMPUTER ======
    GoBildaPinpointDriver odom;

    //====== MECANUM DRIVETRAIN MOTORS (RUN_USING_ENCODER) =====
    protected DcMotorEx frontLeftMotor     = null;
    public int          frontLeftMotorTgt  = 0;       // RUN_TO_POSITION target encoder count
    public int          frontLeftMotorPos  = 0;       // current encoder count
    public double       frontLeftMotorVel  = 0.0;     // encoder counts per second
    public double       frontLeftMotorAmps = 0.0;     // current power draw (Amps)

    protected DcMotorEx frontRightMotor    = null;
    public int          frontRightMotorTgt = 0;       // RUN_TO_POSITION target encoder count
    public int          frontRightMotorPos = 0;       // current encoder count
    public double       frontRightMotorVel = 0.0;     // encoder counts per second
    public double       frontRightMotorAmps= 0.0;     // current power draw (Amps)

    protected DcMotorEx rearLeftMotor      = null;
    public int          rearLeftMotorTgt   = 0;       // RUN_TO_POSITION target encoder count
    public int          rearLeftMotorPos   = 0;       // current encoder count
    public double       rearLeftMotorVel   = 0.0;     // encoder counts per second
    public double       rearLeftMotorAmps  = 0.0;     // current power draw (Amps)

    protected DcMotorEx rearRightMotor     = null;
    public int          rearRightMotorTgt  = 0;       // RUN_TO_POSITION target encoder count
    public int          rearRightMotorPos  = 0;       // current encoder count
    public double       rearRightMotorVel  = 0.0;     // encoder counts per second
    public double       rearRightMotorAmps = 0.0;     // current power draw (Amps)

    public final static double MIN_DRIVE_POW      = 0.03;    // Minimum speed to move the robot
    public final static double MIN_TURN_POW       = 0.03;    // Minimum speed to turn the robot
    public final static double MIN_STRAFE_POW     = 0.04;    // Minimum speed to strafe the robot
    protected double COUNTS_PER_MOTOR_REV  = 28.0;    // goBilda Yellow Jacket Planetary Gear Motor Encoders
//  protected double DRIVE_GEAR_REDUCTION  = 26.851;  // goBilda 26.9:1 (223rpm) gear ratio with 1:1 HDT5 pully/belt
    protected double DRIVE_GEAR_REDUCTION  = 19.203;  // goBilda 19.2:1 (312rpm) gear ratio with 1:1 HDT5 pully/belt
    protected double MECANUM_SLIPPAGE      = 1.01;    // one wheel revolution doesn't achieve 6" x 3.1415 of travel.
    protected double WHEEL_DIAMETER_INCHES = 3.77953; // (96mm) -- for computing circumference
    protected double COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * MECANUM_SLIPPAGE) / (WHEEL_DIAMETER_INCHES * 3.1415);
    // The math above assumes motor encoders.  For REV odometry pods, the counts per inch is different
    protected double COUNTS_PER_INCH2      = 1738.4;  // 8192 counts-per-rev / (1.5" omni wheel * PI)

    //====== Worm gear pan and tilt MOTORS (RUN_USING_ENCODER) =====
    protected DcMotorEx wormPanMotor       = null;
    public int          wormPanMotorTgt    = 0;       // RUN_TO_POSITION target encoder count
    public int          wormPanMotorPos    = 0;       // current encoder count
    public double       wormPanMotorVel    = 0.0;     // encoder counts per second
    public double       wormPanMotorAmps   = 0.0;     // current power draw (Amps)
    public double       wormPanMotorAmpsPk = 0.0;     // peak power draw (Amps)
    public double       wormPanMotorSetPwr = 0.0;     // requested power setting
    public double       wormPanMotorPwr    = 0.0;     // current power setting

    public double       PAN_ANGLE_HW_MAX    =  200.0;  // encoder angles at maximum rotation RIGHT
    public double       PAN_ANGLE_HW_BASKET =    0.0;  // encoder for rotation back to the basket for scoring
    public double       PAN_ANGLE_HW_MIN    = -575.0;  // encoder angles at maximum rotation LEFT

    protected DcMotorEx wormTiltMotor       = null;
    public int          wormTiltMotorTgt    = 0;      // RUN_TO_POSITION target encoder count
    public int          wormTiltMotorPos    = 0;      // current encoder count
    public double       wormTiltMotorVel    = 0.0;    // encoder counts per second
    public double       wormTiltMotorAmps   = 0.0;    // current power draw (Amps)
    public double       wormTiltMotorAmpsPk = 0.0;    // peak power draw (Amps) 312rpm = 9.2A @ 12V
    public double       wormTiltMotorSetPwr = 0.0;    // requested power setting
    public double       wormTiltMotorPwr    = 0.0;    // current power setting

    protected AnalogInput armTiltEncoder   = null;    // US Digital absolute magnetic encoder (MA3)
    public double           armTiltAngle   = 0.0;     // 0V = 0 degrees; 3.3V = 359.99 degrees
    public static final double armTiltAngleOffset = 129.0;     // allows us to adjust the 0-360 deg range
    public double       turretAngleTarget   = 0.0;     // Automatic movement target angle (degrees)

    public int          TILT_ANGLE_HW_MAX   =  3675;  // encoder at maximum rotation UP/BACK (horizontal = -200)
    public int          TILT_ANGLE_BASKET   =  3675;  // 93.8 deg at 3582 encoder at rotation back to the basket for scoring
    public int          TILT_ANGLE_RAISED   =  2000;  // 54.5 deg encoder at rotation back to the basket for scoring
    public int          TILT_ANGLE_HANG1    =  1400;  // 40.1 deg encoder when preparing for level 2 ascent
    public int          TILT_ANGLE_HANG2    =   400;   // 16.4 deg encoder at the end of level 2 ascent
    public int          TILT_ANGLE_ZERO     =     0;   // 7 deg encoder for parking fullyh reset in auto
    public int          TILT_ANGLE_DRIVE    =   200;   // 11.8 deg encoder for parking in auto or driving around
    public int          TILT_ANGLE_AUTO1    =  2005;  // 54.8 deg tilted up for autonomous specimen scoring (above bar)
    public int          TILT_ANGLE_AUTO2    =  1780;  // 49.6 tilted up for autonomous specimen scoring (clipped)
    public int          TILT_ANGLE_HW_MIN   = -2000;  // does not exist encoder at maximum rotation DOWN/FWD

    // This value is set at init.
    public static double startingArmTiltAngle = 0.0;
    // Delta math from -0.1 deg -3891 encoder counts
    //                  94.4 deg 5 encoder counts
    //                  94.5 deg 3896 encoder counts range
    public final static double ENCODER_COUNTS_PER_DEG  = 3896.0 / 94.5;
    public final static double TILT_ANGLE_HW_MAX_DEG   =   95.00;  // encoder at maximum rotation UP/BACK (horizontal = -200)
    public final static double TILT_ANGLE_BASKET_DEG   =   95.00;  // encoder at rotation back to the basket for scoring
    public final static double TILT_ANGLE_ASCENT1_DEG  =   93.80;  // encoder at rotation back to the low bar for ascent level 1
    public final static double TILT_ANGLE_RAISED_DEG   =   54.50;  // encoder at rotation back to the basket for scoring
    public final static double TILT_ANGLE_HANG1_DEG    =   40.10;  // encoder when preparing for level 2 ascent
    public final static double TILT_ANGLE_HANG2_DEG    =   16.40; // encoder at the end of level 2 ascent
    public final static double TILT_ANGLE_ZERO_DEG     =    7.00; // encoder for parking fullyh reset in auto
    public final static double TILT_ANGLE_DRIVE_DEG    =   11.80; // encoder for parking in auto or driving around
    public final static double TILT_ANGLE_AUTO1_DEG    =   54.80; // tilted up for autonomous specimen scoring (above bar)
    public final static double TILT_ANGLE_AUTO2_DEG    =   49.60; // tilted up for autonomous specimen scoring (clipped)
    public final static double TILT_ANGLE_HW_MIN_DEG   =    0.00; // encoder at maximum rotation DOWN/FWD

    //====== Viper slide MOTOR (RUN_USING_ENCODER) =====
    protected DcMotorEx viperMotor       = null;
    public int          viperMotorTgt    = 0;       // RUN_TO_POSITION target encoder count
    public int          viperMotorPos    = 0;       // current encoder count
    public double       viperMotorVel    = 0.0;     // encoder counts per second
    public double       viperMotorAmps   = 0.0;     // current power draw (Amps)
    public double       viperMotorAmpsPk = 0.0;     // peak power draw (Amps) 312rpm = 9.2A @ 12V
    public double       viperMotorSetPwr = 0.0;     // requested power setting
    public double       viperMotorPwr    = 0.0;     // current power setting

    public ElapsedTime  viperSlideTimer    = new ElapsedTime();
    public boolean      viperMotorAutoMove = false;  // have we commanded an automatic lift movement?
    public boolean      viperMotorBusy     = false;
    public double       VIPER_RAISE_POWER  =  1.000; // Motor power used to EXTEND viper slide
    public double       VIPER_HOLD_POWER   =  0.001; // Motor power used to HOLD viper slide at current extension
    public double       VIPER_LOWER_POWER  = -0.500; // Motor power used to RETRACT viper slide

    // Encoder counts for 435 RPM lift motors theoretical max 5.8 rev * 384.54 ticks/rev = 2230.3 counts
    // Encoder counts for 312 RPM lift motors theoretical max ??? rev * 537.7  ticks/rev = ?? counts
    public int          VIPER_EXTEND_ZERO  = 0;      // fully retracted (may need to be adjustable??)
    public int          VIPER_EXTEND_AUTO  = 482;    // extend for collecting during auto
    public int          VIPER_EXTEND_HANG1 = 2050;   // extend to this to prepare for level 2 ascent
    public int          VIPER_EXTEND_HANG2 = 500;    // retract to this extension during level 2 ascent
    public int          VIPER_EXTEND_GRAB  = 1000;   // extend for collection from submersible
    public int          VIPER_EXTEND_AUTO1 = 1400;   // raised to where the specimen hook is above the high bar
    public int          VIPER_EXTEND_AUTO2 =  980;   // retract to clip the specimen to the bar
    public int          VIPER_EXTEND_BASKET= 3000;   // raised to basket-scoring height
    public int          VIPER_EXTEND_FULL1 = 2000;   // extended 36" forward (max for 20"x42" limit) 2310 with overshoot
    public int          VIPER_EXTEND_FULL2 = 3010;   // hardware fully extended (never exceed this count!)
//  PIDControllerLift   liftPidController;           // PID parameters for the lift motors
//  public double       liftMotorPID_p     = -0.100; //  Raise p = proportional
//  public double       liftMotorPID_i     =  0.000; //  Raise i = integral
//  public double       liftMotorPID_d     = -0.007; //  Raise d = derivative
//  public boolean      liftMotorPIDAuto   = false;  // Automatic movement in progress (PID)
//  public int          liftMotorCycles    = 0;      // Automatic movement cycle count
//  public int          liftMotorWait      = 0;      // Automatic movement wait count (truly there! not just passing thru)
//  public int          liftTarget         = 0;      // Automatic movement target ticks

    //====== COLLECTOR SERVOS =====
    public AnalogInput elbowServoPos = null;
    public Servo elbowServo = null;
    final public static double ELBOW_SERVO_INIT = 0.350;
    final public static double ELBOW_SERVO_INIT_ANGLE = 229.0;
    final public static double ELBOW_SERVO_SAFE = 0.370;  // Safe orientation for driving
    final public static double ELBOW_SERVO_SAFE_ANGLE = 224.0;
    final public static double ELBOW_SERVO_GRAB = 0.340;  // For collecting off the field
    final public static double ELBOW_SERVO_GRAB_ANGLE = 231.0;
    final public static double ELBOW_SERVO_DROP = 0.330;  // For scoring in the basket
    final public static double ELBOW_SERVO_DROP_ANGLE = 234.0;
    final public static double ELBOW_SERVO_BAR1 = 0.525;  // For scoring a specimen on the submersible bar (partial rotate)
    final public static double ELBOW_SERVO_BAR2 = 0.700;  // For scoring a specimen on the sumersible bar (fully rotated)
    final public static double ELBOW_SERVO_BAR_ANGLE = 116.0;

    public AnalogInput wristServoPos = null;
    public Servo  wristServo = null;
    final public static double WRIST_SERVO_INIT = 0.159;          // rotation can hit the floor
    final public static double WRIST_SERVO_INIT_ANGLE = 288.0;
    final public static double WRIST_SERVO_SAFE = 0.340;    // Safe orientation for driving
    final public static double WRIST_SERVO_SAFE_ANGLE = 234.0;
    final public static double WRIST_SERVO_GRAB = 0.860;
    final public static double WRIST_SERVO_GRAB_ANGLE = 67.0;
    final public static double WRIST_SERVO_RAISE = 0.570;    // Safe orientation for driving
    final public static double WRIST_SERVO_RAISE_ANGLE = 157.0;
    final public static double WRIST_SERVO_DROP = 0.350;
    final public static double WRIST_SERVO_DROP_ANGLE = 228.0;
    final public static double WRIST_SERVO_BAR1 = 0.400;
    final public static double WRIST_SERVO_BAR2 = 0.640;
    final public static double WRIST_SERVO_BAR_ANGLE = 134.0;

    public CRServo geckoServo = null;

    //Ultrasonic sensors
//  private MaxSonarI2CXL sonarRangeF = null;

    /* local OpMode members. */
    protected HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware2025Bot(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean isAutonomous ) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Configure REV control/expansion hubs for bulk reads (faster!)
        for (LynxModule module : hwMap.getAll(LynxModule.class)) {
            if(module.isParent()) {
                controlHub = module;
            } else {
                expansionHub = module;
            }
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Locate the odometry controller in our hardware settings
        odom = hwMap.get(GoBildaPinpointDriver.class,"odom");      // Control Hub I2C port 3
        odom.setOffsets(0.0, -48.0);    // odometry pod locations relative center of robot
        odom.setEncoderResolution( GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD ); // 4bar pods
        odom.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        if( isAutonomous ) {
            odom.resetPosAndIMU();
        }

        // Define and Initialize drivetrain motors
        frontLeftMotor  = hwMap.get(DcMotorEx.class,"FrontLeft");  // Expansion Hub port 1 (REVERSE)
        frontRightMotor = hwMap.get(DcMotorEx.class,"FrontRight"); // Expansion Hub port 2 (forward)
        rearLeftMotor   = hwMap.get(DcMotorEx.class,"RearLeft");   // Expansion Hub port 0 (REVERSE)
        rearRightMotor  = hwMap.get(DcMotorEx.class,"RearRight");  // Expansion Hub port 3 (forward)

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);  // goBilda fwd/rev opposite of Matrix motors!
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all drivetrain motors to zero power
        driveTrainMotorsZero();

        // Set all drivetrain motors to run with encoders.
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set all drivetrain motors to brake when at zero power
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and Initialize pan and tilt motors
        wormPanMotor   = hwMap.get(DcMotorEx.class,"WormPan");   // Control Hub port 0
        wormTiltMotor  = hwMap.get(DcMotorEx.class,"WormTilt");  // Control Hub port 1
        armTiltEncoder = hwMap.get(AnalogInput.class, "tiltMA3"); // Expansion Hub analog 0
        startingArmTiltAngle = computeAbsoluteAngle( armTiltEncoder.getVoltage(), armTiltAngleOffset);

        wormPanMotor.setDirection(DcMotor.Direction.FORWARD);
        wormTiltMotor.setDirection(DcMotor.Direction.FORWARD);

        wormPanMotor.setPower( 0.0 );
        wormTiltMotor.setPower( 0.0 );

        if( isAutonomous ) {
            wormPanMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            wormPanMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } // not for teleop

        // Reset tilt encoder so we can base everything off the absolute position encoder
        wormTiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormTiltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wormPanMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wormTiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Viper slide motor
        viperMotor = hwMap.get(DcMotorEx.class,"viperMotor");  // Expansion Hub port 2
        viperMotor.setDirection(DcMotor.Direction.REVERSE);   // positive motor power extends
        viperMotor.setPower( 0.0 );
        if( isAutonomous ) {
            viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            viperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        viperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elbowServo = hwMap.servo.get("ElbowServo");             // servo port 0 (Expansion Hub)
        elbowServoPos = hwMap.analogInput.get("ElbowServoPos"); // Analog port 1 (Expansion Hub)
        elbowServo.setPosition(ELBOW_SERVO_INIT);

        wristServo = hwMap.servo.get("WristServo");             // servo port 1 (Expansion Hub)
        wristServoPos = hwMap.analogInput.get("WristServoPos"); // Analog port 0 (Expansion Hub)
        wristServo.setPosition(WRIST_SERVO_INIT);

        geckoServo = hwMap.crservo.get("GeckoServo");           // servo port 2 (Expansion Hub)
        geckoServo.setPower(0.0);

        // Initialize REV Control Hub IMU
        initIMU();

//      sonarRangeF = hwMap.get( MaxSonarI2CXL.class, "distance" );

    } /* init */

    /*--------------------------------------------------------------------------------------------*/
    public void resetEncoders() {
        wormPanMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormTiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wormPanMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wormTiltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        odom.resetPosAndIMU();
    } // resetEncoders

    /*--------------------------------------------------------------------------------------------*/
    public void initIMU()
    {
        // Define and initialize REV Expansion Hub IMU
        BNO055IMU.Parameters imu_params = new BNO055IMU.Parameters();
        imu_params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu_params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu_params.calibrationDataFile = "BNO055IMUCalibration.json"; // located in FIRST/settings folder
        imu_params.loggingEnabled = false;
        imu_params.loggingTag = "IMU";
        imu_params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize( imu_params );
    } // initIMU()

    /*--------------------------------------------------------------------------------------------*/
    public double headingIMU()
    {
        Orientation angles = imu.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES );
        imuHeadingAngle = angles.firstAngle;
        imuTiltAngle = angles.secondAngle;
        return -imuHeadingAngle;  // degrees (+90 is CW; -90 is CCW)
    } // headingIMU

    /*--------------------------------------------------------------------------------------------*/
    /*public double headingIMUradians()
    {
        Orientation angles = imu.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS );
        double heading = -(double)angles.firstAngle;
        return heading;  // radians (+pi is CW; -pi is CCW)
    } // headingIMUradians*/

    /*--------------------------------------------------------------------------------------------*/
    public boolean isPwrRampingDown( double priorPwr, double thisPwr ) {
        // Check for POSITIVE power ramp-down (0.30 to 0.20)
        boolean positiveRampDown = ((priorPwr >= 0.0) && (thisPwr >= 0.0) && (thisPwr < priorPwr));
        // Check for NEGATIVE power ramp-down (-0.30 to -0.20)
        boolean negativeRampDown = ((priorPwr <= 0.0) && (thisPwr <= 0.0) && (thisPwr > priorPwr));
        // If either is true, we're ramping down power toward zero
        return positiveRampDown || negativeRampDown;
    } // isPwrRampingDown

    /*--------------------------------------------------------------------------------------------*/
    /* NOTE: The absolute magnetic encoders may not be installed with 0deg rotated to the "right" */
    /* rotational angle to put ZERO DEGREES where we want it.  By defining a starting offset, and */
    /* using this function to account for that offset, we can place zero where we want it in s/w. */
    /* Having DEGREES_PER_ROTATION as a variable lets us adjust for the 3.3V vs. 5.0V difference. */
    /*--------------------------------------------------------------------------------------------*/
    public double computeAbsoluteAngle( double measuredVoltage, double zeroAngleOffset )
    {
        final double DEGREES_PER_ROTATION = 360.0;  // One full rotation measures 360 degrees
        final double MAX_MA3_ANALOG_VOLTAGE = 3.3;  // 3.3V maximum analog output
        // NOTE: when vertical the angle is 38.1deg, when horizontal 129.0 (prior to offset below)
        double measuredAngle = (measuredVoltage / MAX_MA3_ANALOG_VOLTAGE) * DEGREES_PER_ROTATION;
        // Correct for the offset angle (see note above)
        double correctedAngle = zeroAngleOffset - measuredAngle;
        // Enforce that any wrap-around remains in the range of -180 to +180 degrees
        while( correctedAngle < -180.0 ) correctedAngle += 360.0;
        while( correctedAngle > +180.0 ) correctedAngle -= 360.0;
        return correctedAngle;
    } // computeAbsoluteAngle

    public double computeRawAngle( double measuredVoltage )
    {
        final double DEGREES_PER_ROTATION = 360.0;  // One full rotation measures 360 degrees
        final double MAX_MA3_ANALOG_VOLTAGE = 3.3;  // 3.3V maximum analog output
        // NOTE: when vertical the angle is 38.1deg, when horizontal 129.0 (prior to offset below)
        double measuredAngle = (measuredVoltage / MAX_MA3_ANALOG_VOLTAGE) * DEGREES_PER_ROTATION;
        // Correct for the offset angle (see note above)
        // Enforce that any wrap-around remains in the range of -180 to +180 degrees
        while( measuredAngle < -180.0 ) measuredAngle += 360.0;
        while( measuredAngle > +180.0 ) measuredAngle -= 360.0;
        return measuredAngle;
    } // computeAbsoluteAngle

    /*--------------------------------------------------------------------------------------------*/
    /* NOTE: The absolute magnetic encoders may not be installed with 0deg rotated to the "right" */
    /* rotational angle to put ZERO DEGREES where we want it.  By defining a starting offset, and */
    /* using this function to account for that offset, we can place zero where we want it in s/w. */
    /* Having DEGREES_PER_ROTATION as a variable lets us adjust for the 3.3V vs. 5.0V difference. */
    /*--------------------------------------------------------------------------------------------*/
    public static int computeEncoderCountsFromAngle( double angle )
    {
        return (int)((angle - startingArmTiltAngle) * ENCODER_COUNTS_PER_DEG);
    } // computeEncoderCountsFromAngle

    /*--------------------------------------------------------------------------------------------*/
    public void readBulkData() {
        // For MANUAL mode, we must clear the BulkCache once per control cycle
        expansionHub.clearBulkCache();
        controlHub.clearBulkCache();
        // Get a fresh set of values for this cycle
        //   getCurrentPosition() / getTargetPosition() / getTargetPositionTolerance()
        //   getPower() / getVelocity() / getCurrent()
        frontLeftMotorPos  = frontLeftMotor.getCurrentPosition();
        frontLeftMotorVel  = frontLeftMotor.getVelocity();
        frontRightMotorPos = frontRightMotor.getCurrentPosition();
        frontRightMotorVel = frontRightMotor.getVelocity();
        rearRightMotorPos  = rearRightMotor.getCurrentPosition();
        rearRightMotorVel  = rearRightMotor.getVelocity();
        rearLeftMotorPos   = rearLeftMotor.getCurrentPosition();
        rearLeftMotorVel   = rearLeftMotor.getVelocity();
        //---- Viper Slide motor data ----
        viperMotorPos      = viperMotor.getCurrentPosition();
        viperMotorVel      = viperMotor.getVelocity();
        viperMotorPwr      = viperMotor.getPower();
        //---- Worm drive PAN motor data ----
        wormPanMotorPos    = wormPanMotor.getCurrentPosition();
        wormPanMotorVel    = wormPanMotor.getVelocity();
        wormPanMotorPwr    = wormPanMotor.getPower();
        //---- Worm drive TILT motor data ----
        wormTiltMotorPos    = wormTiltMotor.getCurrentPosition();
        wormTiltMotorVel    = wormTiltMotor.getVelocity();
        wormTiltMotorPwr    = wormTiltMotor.getPower();
        armTiltAngle = computeAbsoluteAngle( armTiltEncoder.getVoltage(), armTiltAngleOffset);
        // NOTE: motor mA data is NOT part of the bulk-read, so increases cycle time!
//      frontLeftMotorAmps  = frontLeftMotor.getCurrent(MILLIAMPS);
//      frontRightMotorAmps = frontRightMotor.getCurrent(MILLIAMPS);
//      rearRightMotorAmps  = rearRightMotor.getCurrent(MILLIAMPS);
//      rearLeftMotorAmps   = rearLeftMotor.getCurrent(MILLIAMPS);
//      viperMotorAmps      = viperMotor.getCurrent(MILLIAMPS);
//      wormPanMotorAmps    = wormPanMotor.getCurrent(MILLIAMPS);
//      wormTiltMotorAmps   = wormTiltMotor.getCurrent(MILLIAMPS);
    } // readBulkData

    /*--------------------------------------------------------------------------------------------*/
    public void updateAscendMotorAmps() {

        // monitor the viper slide motor current needed to lift robot during ascent       
        viperMotorAmps = viperMotor.getCurrent(CurrentUnit.AMPS);
        if( viperMotorAmps > viperMotorAmpsPk ) viperMotorAmpsPk = viperMotorAmps;

        // monitor the arm tilt motor current needed fold robot up from floor during ascent       
        wormTiltMotorAmps = wormTiltMotor.getCurrent(CurrentUnit.AMPS);
        if( wormTiltMotorAmps > wormTiltMotorAmpsPk ) wormTiltMotorAmpsPk = wormTiltMotorAmps;

        // monitor the arm pan motor current needed to keep arm from rotating during ascent       
        wormPanMotorAmps = wormPanMotor.getCurrent(CurrentUnit.AMPS);
        if( wormPanMotorAmps > wormPanMotorAmpsPk ) wormPanMotorAmpsPk = wormPanMotorAmps;
        
    } // updateAscendMotorAmps

    /*--------------------------------------------------------------------------------------------*/
    // This is a slow operation (involves an I2C reading) so only do it as needed
    public double readBatteryControlHub() {
        // Update local variable and then return that value
        controlHubV = controlHub.getInputVoltage( VoltageUnit.MILLIVOLTS );
        return controlHubV;
    } // readBatteryControlHub
    /*--------------------------------------------------------------------------------------------*/
    // This is a slow operation (involves an I2C reading) so only do it as needed
    public double readBatteryExpansionHub() {
        // Update local variable and then return that value
        expansionHubV = expansionHub.getInputVoltage( VoltageUnit.MILLIVOLTS );
        return expansionHubV;
    } // readBatteryExpansionHub
    /*--------------------------------------------------------------------------------------------*/
    public void driveTrainMotors( double frontLeft, double frontRight, double rearLeft, double rearRight )
    {
        frontLeftMotor.setPower( frontLeft );
        frontRightMotor.setPower( frontRight );
        rearLeftMotor.setPower( rearLeft );
        rearRightMotor.setPower( rearRight );
    } // driveTrainMotors

    /*--------------------------------------------------------------------------------------------*/
    /* Set all 4 motor powers to drive straight FORWARD (Ex: +0.10) or REVERSE (Ex: -0.10)        */
    public void driveTrainFwdRev( double motorPower )
    {
        frontLeftMotor.setPower(  motorPower );
        frontRightMotor.setPower( motorPower );
        rearLeftMotor.setPower(   motorPower );
        rearRightMotor.setPower(  motorPower );
    } // driveTrainFwdRev

    /*--------------------------------------------------------------------------------------------*/
    /* Set all 4 motor powers to strafe RIGHT (Ex: +0.10) or LEFT (Ex: -0.10)                     */
    public void driveTrainRightLeft( double motorPower )
    {
        frontLeftMotor.setPower(   motorPower );
        frontRightMotor.setPower( -motorPower );
        rearLeftMotor.setPower(   -motorPower );
        rearRightMotor.setPower(   motorPower );
    } // driveTrainRightLeft

    /*--------------------------------------------------------------------------------------------*/
    /* Set all 4 motor powers to turn clockwise (Ex: +0.10) or counterclockwise (Ex: -0.10)       */
    public void driveTrainTurn( double motorPower )
    {
        frontLeftMotor.setPower( -motorPower );
        frontRightMotor.setPower( motorPower );
        rearLeftMotor.setPower(  -motorPower );
        rearRightMotor.setPower(  motorPower );
    } // driveTrainTurn

    /*--------------------------------------------------------------------------------------------*/
    public void driveTrainMotorsZero()
    {
        frontLeftMotor.setPower( 0.0 );
        frontRightMotor.setPower( 0.0 );
        rearLeftMotor.setPower( 0.0 );
        rearRightMotor.setPower( 0.0 );
    } // driveTrainMotorsZero

    /*--------------------------------------------------------------------------------------------*/
    public void stopMotion() {
        // Stop all motion;
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
    }

    /*--------------------------------------------------------------------------------------------*/
    public double restrictDeltaPower( double powerNext, double powerNow, double powerStepMax ) {
        // Don't stress our motors (or draw too much current) by going straight from 0% to 100% power!
        double powerDelta = powerNext - powerNow;
        // Is the delta outside the allowed limit?
        // (if so, only step in that direction the allowed amount)
        if( Math.abs(powerDelta) > powerStepMax )
            powerNext = powerNow + ((powerNext > powerNow)? +powerStepMax : -powerStepMax);
        // Return the verified power setting
        return powerNext;
    } // restrictDeltaPower
    
    /*--------------------------------------------------------------------------------------------*/
    /* setRunToPosition()                                                                         */
    /* - driveY -   true = Drive forward/back; false = Strafe right/left                          */
    /* - distance - how far to move (inches).  Positive is FWD/RIGHT                              */
    public void setRunToPosition( boolean driveY, double distance )
    {
        // Compute how many encoder counts achieves the specified distance
        int moveCounts = (int)(distance * COUNTS_PER_INCH);

        // These motors move the same for front/back or right/left driving
        frontLeftMotorTgt  = frontLeftMotorPos  +  moveCounts;
        frontRightMotorTgt = frontRightMotorPos + (moveCounts * ((driveY)? 1:-1));
        rearLeftMotorTgt   = rearLeftMotorPos   + (moveCounts * ((driveY)? 1:-1));
        rearRightMotorTgt  = rearRightMotorPos  +  moveCounts;

        // Configure target encoder count
        frontLeftMotor.setTargetPosition(  frontLeftMotorTgt  );
        frontRightMotor.setTargetPosition( frontRightMotorTgt );
        rearLeftMotor.setTargetPosition(   rearLeftMotorTgt   );
        rearRightMotor.setTargetPosition(  rearRightMotorTgt  );

        // Enable RUN_TO_POSITION mode
        frontLeftMotor.setMode(  DcMotor.RunMode.RUN_TO_POSITION );
        frontRightMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
        rearLeftMotor.setMode(   DcMotor.RunMode.RUN_TO_POSITION );
        rearRightMotor.setMode(  DcMotor.RunMode.RUN_TO_POSITION );
    } // setRunToPosition

    /*--------------------------------------------------------------------------------------------*/
    public double getElbowServoAngle() {
        return (elbowServoPos.getVoltage() / 3.3) * 360.0;
    }
    public double getElbowServoPos()   { return  elbowServo.getPosition(); };

    public double getWristServoAngle() {
        return (wristServoPos.getVoltage() / 3.3) * 360.0;
    }
    public double getWristServoPos()   { return  wristServo.getPosition(); }

    /*--------------------------------------------------------------------------------------------*/
    /* startViperSlideExtension()                                                                 */
    /* NOTE: Comments online say the firmware that executes the motor RUN_TO_POSITION logic want  */
    /* the setup commands in this order: setTargetPosition(), setMode(), setPower().              */
    public void startViperSlideExtension(int targetEncoderCount )
    {
        // Range-check the target
        if( targetEncoderCount < VIPER_EXTEND_ZERO  ) targetEncoderCount = VIPER_EXTEND_ZERO;
        if( targetEncoderCount > VIPER_EXTEND_FULL2 ) targetEncoderCount = VIPER_EXTEND_FULL2;
        // Configure target encoder count
        viperMotor.setTargetPosition( targetEncoderCount );
        // Enable RUN_TO_POSITION mode
        viperMotor.setMode(  DcMotor.RunMode.RUN_TO_POSITION );
        // Are we raising or lowering the lift?
        boolean directionUpward = (targetEncoderCount > viperMotorPos)? true : false;
        // Set the power used to get there (NOTE: for RUN_TO_POSITION, always use a POSITIVE
        // power setting, no matter which way the motor must rotate to achieve that target.
        double motorPower = (directionUpward)? VIPER_RAISE_POWER : -VIPER_LOWER_POWER;
        viperMotor.setPower( motorPower );
        viperSlideTimer.reset();
        // Note that we've started a RUN_TO_POSITION and need to reset to RUN_USING_ENCODER
        viperMotorAutoMove = true;
        viperMotorBusy = true;
    } // startViperSlideExtension

    public void processViperSlideExtension()
    {
        // Has the automatic movement reached its destination?.
        if( viperMotorAutoMove ) {
            if (!viperMotor.isBusy()) {
                viperMotorBusy = false;
            // Timeout reaching destination.
            } else if (viperSlideTimer.milliseconds() > 5000) {
                viperMotorBusy = false;
              //telemetry.addData("processViperSlideExtension", "Movement timed out.");
              //telemetry.addData("processViperSlideExtension", "Position: %d", viperMotor.getCurrentPosition());
              //telemetry.update();
              //telemetrySleep();
            }
        }
    } // processViperSlideExtension
    
    public void abortViperSlideExtension()
    {
        // Have we commanded an AUTOMATIC lift movement that we need to terminate so we
        // can return to MANUAL control?  (NOTE: we don't care here whether the AUTOMATIC
        // movement has finished yet; MANUAL control immediately overrides that action.
        if( viperMotorAutoMove ) {
           // turn off the auto-movement power, but don't go to ZERO POWER or
           // the weight of the lift will immediately drop it back down.
           viperMotor.setMode(  DcMotor.RunMode.RUN_USING_ENCODER );
           viperMotor.setPower( VIPER_HOLD_POWER );
//         liftMoveState = LiftMoveActivity.IDLE;
//         liftStoreState = LiftStoreActivity.IDLE;
           viperMotorAutoMove = false;
           viperMotorBusy = false;
        }
    } // abortViperSlideExtension

    /*--------------------------------------------------------------------------------------------*/
 /*
 
    private int liftMoveTarget;

    private ElapsedTime liftMoveTimer   = new ElapsedTime();
    private ElapsedTime liftStoreTimer  = new ElapsedTime();

    public enum LiftMoveActivity {
        IDLE,
        LIFTING_SAFE,
        LIFTING,
        ROTATING
    }
    public LiftMoveActivity liftMoveState = LiftMoveActivity.IDLE;

    public enum LiftStoreActivity {
        IDLE,
        ROTATING,
        LOWERING
    }
    public LiftStoreActivity liftStoreState = LiftStoreActivity.IDLE;

    //--------------------------------------------------------------------------------------------
    public void startLiftMove(int liftTarget)
    {
        // Ensure pre-conditions are met
        if( (pixelScoreAutoState == PixelScoreAutoActivity.IDLE) &&
            (pixelGrabState == PixelGrabActivity.IDLE) &&
            (pixelScoreState == PixelScoreActivity.IDLE) ) {
            // startViperSlideExtension handles lift power vs lower power
            liftMoveTarget = liftTarget;
            startViperSlideExtension(liftMoveTarget);
            liftMoveTimer.reset();
            liftMoveState = LiftMoveActivity.LIFTING_SAFE;
        }
    } // startLiftMove

    //--------------------------------------------------------------------------------------------

    public void liftMotorSetPower( double motorPower )
    {
        // Are we stopping? (allow that no matter what the current liftAngle or power setting)
        if( Math.abs(motorPower) <= 0.0001 ) {
            viperMotorSetPwr = 0.0;
        }
        else {
            // Limit motor acceleration by clamping how big a change we can do in one cycle
            viperMotorSetPwr = restrictDeltaPower( motorPower, viperMotorSetPwr, 0.333 );
        }
        viperMotor.setPower( viperMotorSetPwr );
    } // liftMotorSetPower

    public void liftPIDPosRun( boolean teleopMode )
    {
        // Has an automatic movement been initiated?
        if(liftMotorPIDAuto) {
            // Keep track of how long we've been doing this
            liftMotorCycles++;
            // Current distance from target (angle degrees)
            int ticksToGo = liftTarget - viperMotorPos;
            int ticksToGoAbs = Math.abs(ticksToGo);
            int waitCycles = (teleopMode) ? 2 : 2;
            double power = liftPidController.update(liftTarget, viperMotorPos);
            liftMotorSetPower(power);
            // Have we achieved the target?
            // (temporarily limit to 16 cycles when verifying any major math changes!)
//          if( liftMotorCycles >= 16 ) {
            if( ticksToGoAbs <= 10 ) {
                liftMotorSetPower( 0.0 );
                if( ++liftMotorWait >= waitCycles ) {
                    liftMotorPIDAuto = false;
//                  writeLiftLog();
                }
            }
            // No, still not within tolerance of desired target
            else {
                // Reset the wait count back to zero
                liftMotorWait = 0;
            }
        } // liftMotorPIDAuto
    } // liftPIDPosRun

    public void processLiftMove() {
        switch(liftMoveState){
            // Make sure we are over the bin to rotate
            case LIFTING_SAFE:
                // We are above the pixel bin and can start rotating
                if(viperMotorPos >= VIPER_EXTEND_GRAB) {
                    // This shouldn't happen, but make sure we aren't moving below
                    // the pixel bin height before starting wrist movement. Moving below
                    // pixel bin should use the store activity.
                    if(liftMoveTarget >= VIPER_EXTEND_GRAB) {
                        wristServo.setPosition(WRIST_SERVO_DROP);
                    }
                    liftMoveTimer.reset();
                    liftMoveState = LiftMoveActivity.LIFTING;
                }
                // This is a timeout, the lift could not get above the bin, we probably
                // want to stop here.
                else if(liftMoveTimer.milliseconds() > 2000.0) {
//                    telemetry.addData("processLiftMove", "Timed out lifting safe");
//                    telemetry.addData("processLiftMove", "Lift Target: %d Lift Position: %d", liftMoveTarget, viperMotorsPos);
//                    telemetry.update();
//                    telemetrySleep();
                    liftMoveTimer.reset();
                    liftMoveState = LiftMoveActivity.IDLE;
                }
                break;
            case LIFTING:
                // We are at our desired height
                if(!viperMotorBusy) {
                    liftMoveTimer.reset();
                    liftMoveState = LiftMoveActivity.ROTATING;
                }
                // This is a timeout, the lift could not get to target, we got above the  bin
                // so we can do our thing, but not sure why we didn't hit target.
                else if(liftMoveTimer.milliseconds() > 5000.0) {
//                    telemetry.addData("processLiftMove", "Timed out lifting");
//                    telemetry.addData("processLiftMove", "Lift Target: %d Lift Position: %d", liftMoveTarget, viperMotorsPos);
//                    telemetry.update();
//                    telemetrySleep();
                    liftMoveTimer.reset();
                    liftMoveState = LiftMoveActivity.ROTATING;
                }
                break;
            case ROTATING:
                // We have rotated to the desired position
                if(getWristServoAngle() <= WRIST_SERVO_DROP_ANGLE) {
                    liftMoveState = LiftMoveActivity.IDLE;
                }
                // This is a timeout, can't think of any other error than angle is wrong? I think for
                // now we proceed as if we rotated.
                else if(liftMoveTimer.milliseconds() > 1000.0) {
//                    telemetry.addData("processLiftMove", "Timed out rotating");
//                    telemetry.addData("processLiftMove", "Wrist: %.2f", getWristServoAngle());
                    // Pull back to the safe/stored position
//                    telemetry.update();
//                    telemetrySleep();
                    liftMoveState = LiftMoveActivity.IDLE;
                }
                break;
            case IDLE:
            default:
                break;
        }
    } // processLiftMove

    public void startLiftStore()
    {
        // Ensure pre-conditions are met
        if(  (pixelScoreAutoState == PixelScoreAutoActivity.IDLE) &&
             (pixelGrabState == PixelGrabActivity.IDLE) &&
             (pixelScoreState == PixelScoreActivity.IDLE) &&
             (viperMotorPos >= VIPER_EXTEND_GRAB)) {
            // Rotate elbow/wrist to safe position
            elbowServo.setPosition(ELBOW_SERVO_SAFE);
            wristServo.setPosition(WRIST_SERVO_SAFE);
            liftStoreTimer.reset();
            liftStoreState = LiftStoreActivity.ROTATING;
        }
    } // startLiftStore

    public void processLiftStore() {
        switch(liftStoreState){
            // Make sure the fingers are in a safe position
            case ROTATING:
                if( (getPushServoAngle() >= PUSH_SERVO_SAFE_ANGLE) &&
                    (getWristServoAngle() >= WRIST_SERVO_GRAB_ANGLE)) {
                    liftStoreState = LiftStoreActivity.LOWERING;
                    startViperSlideExtension(VIPER_EXTEND_ZERO);
                    liftStoreTimer.reset();
                }
                // This is a timeout, the fingers are not safe, abort.
                else if(liftStoreTimer.milliseconds() > 1500.0) {
//                    telemetry.addData("processLiftStore", "Timed out rotating");
//                    telemetry.addData("processLiftStore", "Push: %.2f Wrist: %.2f", getPushServoAngle(), getWristServoAngle());
//                    telemetry.update();
//                    telemetrySleep();
                    liftStoreState = LiftStoreActivity.IDLE;
                }
                break;
            case LOWERING:
                // We are at our desired height
                if(!viperMotorBusy) {
                    liftStoreState = LiftStoreActivity.IDLE;
                }
                // This is a timeout, the lift could not fully lower.
                else if(liftStoreTimer.milliseconds() > 5000.0) {
//                    telemetry.addData("processLiftStore", "Timed out lowering");
//                    telemetry.addData("processLiftStore", "Lift Position: %d", viperMotorsPos);
//                    telemetry.update();
//                    telemetrySleep();
                    liftStoreState = LiftStoreActivity.IDLE;
                }
                break;
            case IDLE:
            default:
                break;
        }
    } // processLiftStore
*/
    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    } /* waitForTick() */

} /* Hardware2025Bot */
