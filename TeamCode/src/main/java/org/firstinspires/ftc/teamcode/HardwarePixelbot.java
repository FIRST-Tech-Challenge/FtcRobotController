package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.MILLIAMPS;
import static java.lang.Thread.sleep;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.HardwareDrivers.MaxSonarI2CXL;

/*
 * Hardware class for goBilda robot (12"x15" chassis with 96mm/3.8" goBilda mecanum wheels)
 */
public class HardwarePixelbot
{
    //====== REV CONTROL/EXPANSION HUBS =====
    LynxModule controlHub;
    LynxModule expansionHub;
    public double controlHubV   = 0.0; // Voltage supply of the control hub
    public double expansionHubV = 0.0; // Voltage supply of the expansion hub

    //====== INERTIAL MEASUREMENT UNIT (IMU) =====
    protected BNO055IMU imu    = null;
    public double headingAngle = 0.0;
    public double tiltAngle    = 0.0;

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
//  protected double COUNTS_PER_INCH2      = 1738.4;  // REV ENCODERS = 8192 counts-per-rev / (1.5" omni wheel * PI)
    protected double COUNTS_PER_INCH2      = 336.878; // goBilda Odometry Pod = 2000 counts-per-rev / (1.8897" omni wheel * PI)

    // Thinnear lift motor for hanging (one motor drives both left/right "Thinnear" linear-actuators)
    protected DcMotorEx   thinnearMotor     = null;
//  public double         thinnearMotorAmps = 0.0;     // current power draw (Amps)
    public DigitalChannel thinnearTopSensor;           // REV magnetic limit switch (placed at top)
    public boolean        thinnearTopLimit = false;
    public DigitalChannel thinnearBottomSensor;        // REV magnetic limit switch (placed at bottom)
    public boolean        thinnearBottomLimit = false;

    // Pixel collector/transporter motor
    protected DcMotorEx collectorMotor     = null;

    public double  COLLECTOR_MOTOR_POWER = 0.90;  // Speed of the collector motor when we run
    public double  COLLECTOR_EJECT_POWER = -0.90;  // Speed of the collector motor for autonomous (assumes ON TOP!)

    // Viper slide motors (Y power cable to drive both motors from one port; single encoder cable on left motor
    protected DcMotorEx viperMotors = null;

    // Potential dual motor port solution
    protected DcMotorEx viperMotor1 = null;
    protected DcMotorEx viperMotor2 = null;
    public double viperMotorsSetPwr = 0.0;         // viper motors set power
    public int          viperMotorsPos  = 0;       // current encoder count
    public double       viperMotorsVel  = 0.0;     // encoder counts per second
    public double       viperMotorsPwr  = 0.0;     // current power setting
    public double       viperMotorsAmps = 0.0;     // current power draw (Amps)

    public boolean      viperMotorAutoMove = false;  // have we commanded an automatic lift movement?
    public boolean      viperMotorBusy = false;
    public double  VIPER_RAISE_POWER =  1.000; // Motor power used to RAISE viper slide
    public double  VIPER_HOLD_POWER  =  0.001; // Motor power used to HOLD viper slide at current height
    public double  VIPER_LOWER_POWER = -0.500; // Motor power used to LOWER viper slide
    /*
    // Encoder counts for 1620 RPM lift motors theoretical max 5.8 rev * 103.76 ticks/rev = 601.8
    public int     VIPER_EXTEND_ZERO = 0;    // 1620 Encoder count when fully retracted (may need to be adjustable??)
    public int     VIPER_EXTEND_AUTO = 130;  // 1620 Encoder count when raised to just above the bin (safe to rotate)
    public int     VIPER_EXTEND_BIN  = 140;  // 1620 Encoder count when raised to just above the bin (safe to rotate)
    public int     VIPER_EXTEND_LOW  = 145;  // 1620 Encoder count when raised to lowest possible scoring position
    public int     VIPER_EXTEND_MID  = 280;  // 1620 Encoder count when raised to medium scoring height
    public int     VIPER_EXTEND_HIGH = 400;  // 1620 Encoder count when raised to upper scoring height
    public int     VIPER_EXTEND_FULL = 580;  // 1620 Encoder count when fully extended (never exceed this count!)
    */
    /*
    // Encoder counts for 1150 RPM lift motors theoretical max 5.8 rev * 145.09 ticks/rev = 841.5
    public int     VIPER_EXTEND_ZERO = 0;    // 1150 Encoder count when fully retracted (may need to be adjustable??)
    public int     VIPER_EXTEND_AUTO = 182;  // 1150 Encoder count when raised to just above the bin (safe to rotate)
    public int     VIPER_EXTEND_BIN  = 196;  // 1150 Encoder count when raised to just above the bin (safe to rotate)
    public int     VIPER_EXTEND_LOW  = 203;  // 1150 Encoder count when raised to lowest possible scoring position
    public int     VIPER_EXTEND_MID  = 392;  // 1150 Encoder count when raised to medium scoring height
    public int     VIPER_EXTEND_HIGH = 559;  // 1150 Encoder count when raised to upper scoring height
    public int     VIPER_EXTEND_FULL = 811;  // 1150 Encoder count when fully extended (never exceed this count!)
     */
    // Encoder counts for 435 RPM lift motors theoretical max 5.8 rev * 384.54 ticks/rev = 2230.3
    public int     VIPER_EXTEND_ZERO = 0;    // 435 Encoder count when fully retracted (may need to be adjustable??)
    public int     VIPER_EXTEND_AUTO = 482;  // 435 Encoder count when raised to just above the bin (safe to rotate)
    public int     VIPER_EXTEND_BIN  = 519;  // 435 Encoder count when raised to just above the bin (safe to rotate)
    public int     VIPER_EXTEND_LOW  = 537;  // 435 Encoder count when raised to lowest possible scoring position
    public int     VIPER_EXTEND_MID  = 1038; // 435 Encoder count when raised to medium scoring height
    public int     VIPER_EXTEND_HIGH = 1482; // 435 Encoder count when raised to upper scoring height
    public int     VIPER_EXTEND_FULL = 2149; // 435 Encoder count when fully extended (never exceed this count!)
    PIDControllerLift liftPidController;           // PID parameters for the lift motors
    public double        liftMotorPID_p   = -0.100;  //  Raise p = proportional
    public double        liftMotorPID_i   =  0.000;  //  Raise i = integral
    public double        liftMotorPID_d   = -0.007;  //  Raise d = derivative
    public boolean      liftMotorPIDAuto   = false;   // Automatic movement in progress (PID)
    public int          liftMotorCycles    = 0;       // Automatic movement cycle count
    public int          liftMotorWait      = 0;       // Automatic movement wait count (truly there! not just passing thru)
    public int       liftTarget    = 0;     // Automatic movement target ticks

    //====== SERVO FOR COLLECTOR ARM ====================================================================
    public Servo  collectorServo       = null;

    public double COLLECTOR_SERVO_GROUND = 0.910;
    public double COLLECTOR_SERVO_STACK2 = 0.780;
    public double COLLECTOR_SERVO_STACK3 = 0.750;
    public double COLLECTOR_SERVO_STACK4 = 0.710;
    public double COLLECTOR_SERVO_STACK5 = 0.680;
    public double COLLECTOR_SERVO_RAISED = 0.440;  // almost vertical
    public double COLLECTOR_SERVO_STORED = 0.330;  // past this hits the collector motor

    public double collectorServoSetPoint = COLLECTOR_SERVO_STORED;

    public int collectorServoIndex = 1;

    public boolean collectorServoChanged = false;

    //====== COLOR/DISTANCE SENSORS FOR PIXEL BIN ========================================================
    private NormalizedColorSensor pixel1ColorSensor = null;  // lower
    private NormalizedColorSensor pixel2ColorSensor = null;  // upper

    private NormalizedRGBA  pixelRGBA;
    private float[]         pixelHSV = new float[3];

    public double pixel1Hue; // lower
    public double pixel2Hue; // upper
    enum PixelColorsEnum {
        EMPTY,
        WHITE,
        YELLOW,
        GREEN,
        PURPLE,
        UNKNOWN
    }
    public PixelColorsEnum pixel1Color = PixelColorsEnum.EMPTY;  // lower
    public PixelColorsEnum pixel2Color = PixelColorsEnum.EMPTY;  // upper
    private DistanceSensor pixe1DistanceSensor = null;        // lower
    private DistanceSensor pixe2DistanceSensor = null;        // upper

    public double pixel1Distance; // lower
    public double pixel2Distance; // upper

    public DigitalChannel lPixelRed = null;
    public DigitalChannel lPixelGreen = null;
    public DigitalChannel rPixelRed = null;
    public DigitalChannel rPixelGreen = null;

    //====== SERVOS FOR PIXEL FINGERS ====================================================================
    public boolean scorePixelGo = false;
    public AnalogInput pushServoPos = null;
    public Servo  pushServo = null;
    public double PUSH_SERVO_INIT = 0.470;
    final public static double PUSH_SERVO_INIT_ANGLE = 188.5;
    public double PUSH_SERVO_SAFE = 0.470;  // Retract linkage servo back behind the pixel bin (safe to raise/lower)
    final public static double PUSH_SERVO_SAFE_ANGLE = 184.0;
    public double PUSH_SERVO_GRAB = 0.540;  // Partially extend to align fingers inside pixels
    final public static double PUSH_SERVO_GRAB_ANGLE = 168.0;
    public double PUSH_SERVO_DROP = 0.890;  // Fully extend finger assembly toward the Backdrop
    final public static double PUSH_SERVO_DROP_ANGLE = 56.1;
    final public static double PUSH_SERVO_PIXEL_CLEAR_ANGLE = 90.0; // Pulling back from backdrop but cleared pixel

    public AnalogInput wristServoPos = null;
    public Servo  wristServo = null;
    public double WRIST_SERVO_INIT = 0.450;   // higher is counter-clockwise
    final public static double WRIST_SERVO_INIT_ANGLE = 188.0; // no idea yet, will have to figure it out!
    public double WRIST_SERVO_GRAB = 0.450;
    final public static double WRIST_SERVO_GRAB_ANGLE = 183.5;
    public double WRIST_SERVO_DROP = 0.810;
    final public static double WRIST_SERVO_DROP_ANGLE = 128.9;

    public AnalogInput fingerServo1Pos = null;
	public Servo  fingerServo1 = null;  // TOP (bin) or RIGHT (backdrop)
    public double FINGER1_SERVO_DROP = 0.500;
    final public static double FINGER1_SERVO_DROP_ANGLE = 175.0;
    public double FINGER1_SERVO_GRAB = FINGER1_SERVO_DROP + 0.262; // 0.762

    // Increased the "grabbed" angle to account for pixel variation
    final public static double FINGER1_SERVO_GRAB_ANGLE = 104.0;

    public AnalogInput fingerServo2Pos = null;
	public Servo  fingerServo2 = null;  // BOTTOM (bin) or LEFT (backdrop)
    public double FINGER2_SERVO_DROP = 0.480;
    final public static double FINGER2_SERVO_DROP_ANGLE = 181.0;
    public double FINGER2_SERVO_GRAB = FINGER2_SERVO_DROP + 0.262;  // 0.742
    // Increased the "grabbed" angle to account for pixel variation
    final public static double FINGER2_SERVO_GRAB_ANGLE = 104.0;

    //====== ODOMETRY ENCODERS (encoder values only!) =====
    protected DcMotorEx rightOdometer      = null;
    public int          rightOdometerCount = 0;       // current encoder count
    public int          rightOdometerPrev  = 0;       // previous encoder count

    protected DcMotorEx leftOdometer       = null;
    public int          leftOdometerCount  = 0;       // current encoder count
    public int          leftOdometerPrev   = 0;       // previous encoder count

    protected DcMotorEx strafeOdometer      = null;
    public int          strafeOdometerCount = 0;      // current encoder count
    public int          strafeOdometerPrev  = 0;      // previous encoder count

    //Ultrasonic sensors
    private MaxSonarI2CXL sonarRangeF = null;

    private AnalogInput backdropRange = null;

    /* local OpMode members. */
    protected HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();
    private Telemetry telemetry;

    /* Constructor */
    public HardwarePixelbot(Telemetry telem){
        telemetry = telem;
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

        // Define and Initialize drivetrain motors
        frontRightMotor = hwMap.get(DcMotorEx.class,"FrontRight"); // Control Hub  port 0 (forward)
        rearRightMotor  = hwMap.get(DcMotorEx.class,"RearRight");  // Control Hub  port 1 (forward)
        frontLeftMotor  = hwMap.get(DcMotorEx.class,"FrontLeft");  // Expansion Hub port 0 (REVERSE)
        rearLeftMotor   = hwMap.get(DcMotorEx.class,"RearLeft");   // Expansion Hub port 1 (REVERSE)

        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);  // goBilda fwd/rev opposite of Matrix motors!
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);

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

        collectorMotor  = hwMap.get(DcMotorEx.class,"CollectorMotor");  // Expansion Hub port 2 (REVERSE)
        collectorMotor.setDirection(DcMotor.Direction.REVERSE);
        collectorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collectorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        collectorMotor.setPower( 0.0 );

        collectorServo = hwMap.servo.get("collectorServo");      // servo port 4 (Expansion Hub)
        collectorServo.setPosition(collectorServoSetPoint);

        viperMotors = hwMap.get(DcMotorEx.class,"ViperMotors");  // Control Hub port 2
        viperMotors.setDirection(DcMotor.Direction.FORWARD);
        viperMotors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperMotors.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperMotors.setPower( 0.0 );

        thinnearMotor = hwMap.get(DcMotorEx.class,"ThinnearMotor");     // Control Hub port 3
        thinnearMotor.setDirection(DcMotor.Direction.FORWARD);
        thinnearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thinnearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        thinnearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        thinnearMotor.setPower( 0.0 );

        thinnearTopSensor    = hwMap.get( DigitalChannel.class, "MagneticTop" );     // Expansion Hub Digital port 0-1
        thinnearTopSensor.setMode(DigitalChannel.Mode.INPUT);

        thinnearBottomSensor = hwMap.get( DigitalChannel.class, "MagneticBottom" );  // Control Hub Digital port 0-1
        thinnearBottomSensor.setMode(DigitalChannel.Mode.INPUT);

//      rightOdometer  = hwMap.get(DcMotorEx.class,"OdomRight");   // Control Hub port 3
//      rightOdometer.setDirection(DcMotor.Direction.FORWARD);
//      rightOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//      rightOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//      rightOdometer.setPower( 0.0 );

        // "OdomRight" = overloaded onto thinnearMotor on Control Hub port 3
        // "OdomLeft"  = overloaded onto CollectorMotor on Expansion Hub port 2

        strafeOdometer = hwMap.get(DcMotorEx.class,"OdomStrafe");  // Expansion Hub port 3 (encoder only; no motor)
        strafeOdometer.setDirection(DcMotor.Direction.FORWARD);
        strafeOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeOdometer.setPower( 0.0 );

        /*--------------------------------------------------------------------------------------------*/
        pixel1ColorSensor = hwMap.get(NormalizedColorSensor.class, "BinLowerSensor");  // ControlHub I2C Bus1
        pixel1ColorSensor.setGain(3.0f);
        pixe1DistanceSensor = (DistanceSensor) pixel1ColorSensor;

        pixel2ColorSensor = hwMap.get(NormalizedColorSensor.class, "BinUpperSensor");  // ControlHub I2C Bus2
        pixel2ColorSensor.setGain(3.0f);
        pixe2DistanceSensor = (DistanceSensor) pixel2ColorSensor;

        /*--------------------------------------------------------------------------------------------*/
        pushServo = hwMap.servo.get("ElbowServo");             // servo port 0 (Expansion Hub)
        pushServoPos = hwMap.analogInput.get("ElbowServoPos"); // Analog port 1 (Expansion Hub)
        pushServo.setPosition(PUSH_SERVO_INIT);

        wristServo = hwMap.servo.get("WristServo");             // servo port 1 (Expansion Hub)
        wristServoPos = hwMap.analogInput.get("WristServoPos"); // Analog port 0 (Expansion Hub)
        wristServo.setPosition(WRIST_SERVO_INIT);

        fingerServo1 = hwMap.servo.get("Finger1Servo");             // servo port 2 (Expansion Hub)
        fingerServo1Pos = hwMap.analogInput.get("Finger1ServoPos"); // Analog port 3 (Expansion Hub)
        fingerServo1.setPosition(FINGER1_SERVO_DROP);

        fingerServo2 = hwMap.servo.get("Finger2Servo");             // servo port 3 (Expansion Hub)
        fingerServo2Pos = hwMap.analogInput.get("Finger2ServoPos"); // Analog port 2 (Expansion Hub)
        fingerServo2.setPosition(FINGER2_SERVO_DROP);

        // IR Backdrop Range Sensor
        backdropRange = hwMap.analogInput.get("BackdropRange"); // Analog port 1 (Control Hub)

        // Pixel indicators
        lPixelRed = hwMap.digitalChannel.get("lPixelRed"); // Digital port 4 (Control Hub)
        lPixelRed.setMode(DigitalChannel.Mode.OUTPUT);
        lPixelRed.setState(true); // off
        lPixelGreen = hwMap.digitalChannel.get("lPixelGreen"); // Digital port 5 (Control Hub)
        lPixelGreen.setMode(DigitalChannel.Mode.OUTPUT);
        lPixelGreen.setState(true); // off
        rPixelRed = hwMap.digitalChannel.get("rPixelRed"); // Digital port 6 (Control Hub)
        rPixelRed.setMode(DigitalChannel.Mode.OUTPUT);
        rPixelRed.setState(true); // off
        rPixelGreen = hwMap.digitalChannel.get("rPixelGreen"); // Digital port 7 (Control Hub)
        rPixelGreen.setMode(DigitalChannel.Mode.OUTPUT);
        rPixelGreen.setState(true); // off

        // Initialize REV Control Hub IMU
        initIMU();

//      sonarRangeF = hwMap.get( MaxSonarI2CXL.class, "distance" );

    } /* init */

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
        headingAngle = angles.firstAngle;
        tiltAngle = angles.secondAngle;
        return -headingAngle;  // degrees (+90 is CW; -90 is CCW)
    } // headingIMU

    /*--------------------------------------------------------------------------------------------*/
    public double headingIMUradians()
    {
        Orientation angles = imu.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS );
        double heading = -(double)angles.firstAngle;
        return heading;  // radians (+pi is CW; -pi is CCW)
    } // headingIMUradians

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
        double measuredAngle = (measuredVoltage / MAX_MA3_ANALOG_VOLTAGE) * DEGREES_PER_ROTATION;
        // Correct for the offset angle (see note above)
        double correctedAngle = measuredAngle - zeroAngleOffset;
        // Enforce that any wrap-around remains in the range of -180 to +180 degrees
        while( correctedAngle < -180.0 ) correctedAngle += 360.0;
        while( correctedAngle > +180.0 ) correctedAngle -= 360.0;
        return correctedAngle;
    } // computeAbsoluteAngle

    /*--------------------------------------------------------------------------------------------*/
    public void readBulkData() {
        // For MANUAL mode, we must clear the BulkCache once per control cycle
        expansionHub.clearBulkCache();
        controlHub.clearBulkCache();
        // Copy the bulkdata readings for this cycle into our local variables
        frontLeftMotorPos   = frontLeftMotor.getCurrentPosition();
        frontLeftMotorVel   = frontLeftMotor.getVelocity();
        frontRightMotorPos  = frontRightMotor.getCurrentPosition();
        frontRightMotorVel  = frontRightMotor.getVelocity();
        rearRightMotorPos   = rearRightMotor.getCurrentPosition();
        rearRightMotorVel   = rearRightMotor.getVelocity();
        rearLeftMotorPos    = rearLeftMotor.getCurrentPosition();
        rearLeftMotorVel    = rearLeftMotor.getVelocity();
        viperMotorsPos      = viperMotors.getCurrentPosition();
        viperMotorsVel      = viperMotors.getVelocity();
        viperMotorsPwr      = viperMotors.getPower();
        rightOdometerPrev   = rightOdometerCount;
//      rightOdometerCount  = -rightOdometer.getCurrentPosition(); // Must be POSITIVE when bot moves FORWARD
        rightOdometerCount  = -thinnearMotor.getCurrentPosition(); // Must be POSITIVE when bot moves FORWARD
        leftOdometerPrev    = leftOdometerCount;
        leftOdometerCount   = -collectorMotor.getCurrentPosition();   // Must be POSITIVE when bot moves FORWARD
        strafeOdometerPrev  = strafeOdometerCount;
        strafeOdometerCount = -strafeOdometer.getCurrentPosition();  // Must be POSITIVE when bot moves RIGHT
        thinnearTopLimit    = thinnearTopSensor.getState();
        thinnearBottomLimit = thinnearBottomSensor.getState();
        // NOTE: motor mA data is NOT part of the bulk-read, so increases cycle time!
//      frontLeftMotorAmps  = frontLeftMotor.getCurrent(MILLIAMPS);
//      frontRightMotorAmps = frontRightMotor.getCurrent(MILLIAMPS);
//      rearRightMotorAmps  = rearRightMotor.getCurrent(MILLIAMPS);
//      rearLeftMotorAmps   = rearLeftMotor.getCurrent(MILLIAMPS);
        viperMotorsAmps     = viperMotors.getCurrent(MILLIAMPS);
//      thinnearMotorAmps   = thinnearMotor.getCurrent(MILLIAMPS);
    } // readBulkData

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
    /* Uses color/distance sensors to determine if we have 0, 1, or 2 pixels, plus their color    */
    /*--------------------------------------------------------------------------------------------*/
    public int pixelBinUpdateStatus() {
        final double MAX_DIST_OCCUPIED = 60.0; // minimum distance sensor can report is 54.52mm
        int binPixelCount = 0;
        //========== LOWER POSITION IN PIXEL BIN ==========
        // 1) does the proximity detector detect a pixel?
        pixel1Distance = pixe1DistanceSensor.getDistance(DistanceUnit.MM);
        boolean binPosition1Empty = Double.isNaN(pixel1Distance) || (pixel1Distance > MAX_DIST_OCCUPIED);
        if( binPosition1Empty ) {
            pixel1Hue = 0.0;
            pixel1Color = PixelColorsEnum.EMPTY;
        } // binPositionIsEmpty
        else {
            binPixelCount++;
            // 2) read the color sensor in RGBA mode (red/green/blue/alpha)
            pixelRGBA = pixel1ColorSensor.getNormalizedColors();
            // 3) Convert RGBA to a color integer, and then to HSV (hue/sat/value)
            Color.colorToHSV( pixelRGBA.toColor(), pixelHSV );
            // 4) Store the HUE
            pixel1Hue = pixelHSV[0];
            // 5) Convert to an enumerated pixel color using hue & saturation
            pixel1Color = pixelHueSatToColor( pixelHSV[0], pixelHSV[1] );
        } // !binPositionIsEmpty
        //========== UPPER POSITION IN PIXEL BIN ==========
        // 1) does the proximity detector detect a pixel?
        pixel2Distance = pixe2DistanceSensor.getDistance(DistanceUnit.MM);
        boolean binPosition2Empty = Double.isNaN(pixel2Distance) || (pixel2Distance > MAX_DIST_OCCUPIED);
        if( binPosition2Empty ) {
            pixel2Hue = 0.0;
            pixel2Color = PixelColorsEnum.EMPTY;
        } // binPositionIsEmpty
        else {
            binPixelCount++;
            // 2) read the color sensor in RGBA mode (red/green/blue/alpha)
            pixelRGBA = pixel2ColorSensor.getNormalizedColors();
            // 3) Convert RGBA to a color integer, and then to HSV (hue/sat/value)
            Color.colorToHSV( pixelRGBA.toColor(), pixelHSV );
            // 4) Store the HUE
            pixel2Hue = pixelHSV[0];
            // 5) Convert to an enumerated pixel color using hue & saturation
            pixel2Color = pixelHueSatToColor( pixelHSV[0], pixelHSV[1] );
        }
        return binPixelCount;
    } // pixelBinUpdateStatus

    /*--------------------------------------------------------------------------------------------*/
    /* Analyzes Hue & Saturation to decide if we have a YELLOW, GREEN, PURPLE, or WHITE pixel.    */
    /*--------------------------------------------------------------------------------------------*/
    private PixelColorsEnum pixelHueSatToColor( double hue, double sat ) {
        PixelColorsEnum pixelColor;
        if((hue > 20.0) && (hue <= 65.0))
            pixelColor = PixelColorsEnum.YELLOW; // hue typically 45-48
        else if((hue > 70.0) && (hue <= 180.0)) {
            if( sat > 0.350 )
                pixelColor = PixelColorsEnum.GREEN;  // hue typically 109-117, saturation 0.560
            else
                pixelColor = PixelColorsEnum.WHITE;  // hue typically 100-142, saturation 0.160
        }
        else if((hue > 200.0) && (hue <= 250.0))
            pixelColor = PixelColorsEnum.PURPLE;  // hue typically 216-225
        else
            pixelColor = PixelColorsEnum.UNKNOWN;
        return pixelColor;
    } // pixelHueSatToColor

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

    public ElapsedTime viperSlideTimer = new ElapsedTime();
    /*--------------------------------------------------------------------------------------------*/
    /* viperSlideExtension()                                                                      */
    /* NOTE: Comments online say the firmware that executes the motor RUN_TO_POSITION logic want  */
    /* the setup commands in this order: setTargetPosition(), setMode(), setPower().              */
    public void startViperSlideExtension(int targetEncoderCount )
    {
        // Range-check the target
        if( targetEncoderCount < VIPER_EXTEND_ZERO ) targetEncoderCount = VIPER_EXTEND_ZERO;
        if( targetEncoderCount > VIPER_EXTEND_FULL ) targetEncoderCount = VIPER_EXTEND_FULL;
        // Configure target encoder count
        viperMotors.setTargetPosition( targetEncoderCount );
        // Enable RUN_TO_POSITION mode
        viperMotors.setMode(  DcMotor.RunMode.RUN_TO_POSITION );
        // Are we raising or lowering the lift?
        boolean directionUpward = (targetEncoderCount > viperMotorsPos)? true : false;
        // Set the power used to get there (NOTE: for RUN_TO_POSITION, always use a POSITIVE
        // power setting, no matter which way the motor must rotate to achieve that target.
        double motorPower = (directionUpward)? VIPER_RAISE_POWER : -VIPER_LOWER_POWER;
        viperMotors.setPower( motorPower );
        viperSlideTimer.reset();
        // Note that we've started a RUN_TO_POSITION and need to reset to RUN_USING_ENCODER
        viperMotorAutoMove = true;
        viperMotorBusy = true;
    } // viperSlideExtension

    public void processViperSlideExtension()
    {
        // Has the automatic movement reached its destination?.
        if( viperMotorAutoMove ) {
            if (!viperMotors.isBusy()) {
                viperMotorBusy = false;
                // Timeout reaching destination.
            } else if (viperSlideTimer.milliseconds() > 5000) {
                viperMotorBusy = false;
                telemetry.addData("processViperSlideExtension", "Movement timed out.");
                telemetry.addData("processViperSlideExtension", "Position: %d", viperMotors.getCurrentPosition());
                telemetry.update();
                telemetrySleep();
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
           viperMotors.setMode(  DcMotor.RunMode.RUN_USING_ENCODER );
           viperMotors.setPower( VIPER_HOLD_POWER );
            liftMoveState = LiftMoveActivity.IDLE;
            liftStoreState = LiftStoreActivity.IDLE;
           viperMotorAutoMove = false;
           viperMotorBusy = false;
        }
    } // abortViperSlideExtension

    /*--------------------------------------------------------------------------------------------*/
    /* NOTE ABOUT RANGE SENSORS:                                                                  */
    /* The REV 2m Range Sensor (5cm-200cm) is really only 1.2m (47.2") maximum in DEFAULT mode.   */
    /* Depending on the reflectivity of the surface encountered, it can be even shorter.  For     */
    /* example, the black metal paint on the field wall is highly absorptive, so we only get      */
    /* reliable range readings out to 30cm/12" or so.  In contrast, the Maxbotics ultrasonic      */
    /* range sensors have a minimum range of 20cm/8". A combined Autonomous solution that requires*/
    /* both short (< 8") and long (> 12-47") requires both REV Time-of-Flight (tof) range sensors */
    /* and Maxbotics Ultrasonic range sensors. Also note that if you mount either ToF/Ultrasonic  */
    /* sensor too low on the robot you'll get invalid distance readings due to reflections off the*/
    /* field tiles due to "fanout" of both laser/ultrasonic signals the further you get from the  */
    /* robot.                                                                                     */
    /*--------------------------------------------------------------------------------------------*/

    public int singleSonarRangeF() {
        //Query the current range sensor reading and wait for a response
        return sonarRangeF.getDistanceSync();
    } // singleSonarRangeF

    enum UltrasonicsInstances
    {
        SONIC_RANGE_LEFT,
        SONIC_RANGE_RIGHT,
        SONIC_RANGE_FRONT,
        SONIC_RANGE_BACK;
    }

    enum UltrasonicsModes
    {
        SONIC_FIRST_PING,
        SONIC_MOST_RECENT;
    }
    public int slowSonarRange( UltrasonicsInstances sensorInstance ) {
        // This is the SLOW version that sends an ultrasonic ping, waits 50 msec for the result,
        // and returns a value.  The returned valued is based on SINGLE reading (no averaging).
        // This version is intended for 1-time readings where the 50msec is okay (not control loops).
        int cm = 0;
        switch( sensorInstance ) {
            case SONIC_RANGE_FRONT : cm = sonarRangeF.getDistanceSync(); break;
            default                : cm = 0;
        } // switch()
        return cm;
    } // slowSonarRange

    public int fastSonarRange( UltrasonicsInstances sensorInstance, UltrasonicsModes mode ) {
        // This is the FAST version that assumes there's a continuous sequence of pings being
        // triggered so it simply returns the "most recent" answer (no waiting!). This function is
        // intended for control loops that can't afford to incur a 50msec delay in the loop time.
        // The first call should pass SONIC_FIRST_PING and ignore the result; All subsequent calls
        // (assuming at least 50 msec has elapsed) should pass SONIC_MOST_RECENT and use the distance
        // returned.
        int cm = 0;
        switch( sensorInstance ) {
            case SONIC_RANGE_FRONT : cm = sonarRangeF.getDistanceAsync(); break;
            default                : cm = 0;
        } // switch()
        // Do we need to zero-out the value returned (likely from another time/place)?
        if( mode == HardwarePixelbot.UltrasonicsModes.SONIC_FIRST_PING ) {
            cm = 0;
        }
        // Return
        return cm;
    } // fastSonarRange

    public double getWristServoAngle() {
        return (wristServoPos.getVoltage() / 3.3) * 360.0;
    }

    public double getPushServoAngle() {
        return (pushServoPos.getVoltage() / 3.3) * 360.0;
    }

    public double getFingerServo1Angle() {
        return (fingerServo1Pos.getVoltage() / 3.3) * 360.0;
    }

    public double getFingerServo2Angle() {
        return (fingerServo2Pos.getVoltage() / 3.3) * 360.0;
    }

    // Returns distance in CM, might have to tweak 80.0 CM as max range to get accurate readings.
    public double getBackdropRange() { return ( (3.3 - backdropRange.getVoltage()) / 3.3) * 30.0; }

    public void setDetectedPixels(int numPixels) {
        switch(numPixels) {
            case 1:
                lPixelRed.setState(false);
                rPixelRed.setState(false);
                lPixelGreen.setState(true);
                rPixelGreen.setState(true);
                break;
            case 2:
                lPixelRed.setState(true);
                rPixelRed.setState(true);
                lPixelGreen.setState(false);
                rPixelGreen.setState(false);
                break;
            default:
                lPixelRed.setState(true);
                rPixelRed.setState(true);
                lPixelGreen.setState(true);
                rPixelGreen.setState(true);
                break;
        }
    }

    /*--------------------------------------------------------------------------------------------*/

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

    /*--------------------------------------------------------------------------------------------*/
    public void liftMotorsSetPower( double motorPower )
    {
        // Are we stopping? (allow that no matter what the current liftAngle or power setting)
        if( Math.abs(motorPower) <= 0.0001 ) {
            viperMotorsSetPwr = 0.0;
        }
        else {
            // Limit motor acceleration by clamping how big a change we can do in one cycle
            viperMotorsSetPwr = restrictDeltaPower( motorPower, viperMotorsSetPwr, 0.333 );
        }
        viperMotor1.setPower( viperMotorsSetPwr );
        viperMotor2.setPower( viperMotorsSetPwr );
    } // liftMotorsSetPower

    /*--------------------------------------------------------------------------------------------*/
    /* liftPIDPosInit()                                                                           */
    /* - newAngle = desired lift angle                                                            */
    public void liftPIDPosInit( int newPosition )
    {
        // 120mm per rotation, 5.8 motor rotations
        // 1150 rpm motor = 145.1 ticks per rotation
        // 841 ticks total, but we have listed 580
        // starting with 580 ticks for 27.4" extension
        // or 21.17 ticks per inch
        // Current distance from target (degrees)
        int ticksToGo = newPosition - viperMotorsPos;
        double pStaticLift  = 0.007;
        // Voltage doesn't seem as important on the arm minimum. Probably don't have to do
        // interpolated voltage. For example 0.13 power was not able to move arm at low voltage
        // and also could not at fresh battery voltage. 0.131 was able to at low voltage.
        // pStaticLower 0.130 @ 12.54V
        // pStaticLift  0.320 @ 12.81V
//        double pSin = getInterpolatedMinPower();

        liftPidController = new PIDControllerLift( liftMotorPID_p, liftMotorPID_i, liftMotorPID_d,
                pStaticLift );

        // Are we ALREADY at the specified ticks?
        // +/- 10 ticks is about +/-0.5" error
        if( Math.abs(ticksToGo) <= 10 )
            return;

        liftPidController.reset();

        // Ensure motor is stopped/stationary (aborts any prior unfinished automatic movement)
        liftMotorsSetPower( 0.0 );

        // Establish a new target angle & reset counters
        liftMotorPIDAuto = true;
        liftTarget = newPosition;
        liftMotorCycles = 0;
        liftMotorWait   = 0;

        // If logging instrumentation, begin a new dataset now:
//        if( liftMotorLogging ) {
//            turretMotorLogVbat = readBatteryExpansionHub();
//            liftMotorLogIndex  = 0;
//            liftMotorLogEnable = true;
//            liftMotorTimer.reset();
//        }

    } // liftPIDPosInit

    /*--------------------------------------------------------------------------------------------*/
    /* liftPIDPosRun()                                                                            */
    public void liftPIDPosRun( boolean teleopMode )
    {
        // Has an automatic movement been initiated?
        if(liftMotorPIDAuto) {
            // Keep track of how long we've been doing this
            liftMotorCycles++;
            // Current distance from target (angle degrees)
            int ticksToGo = liftTarget - viperMotorsPos;
            int ticksToGoAbs = Math.abs(ticksToGo);
            int waitCycles = (teleopMode) ? 2 : 2;
            double power = liftPidController.update(liftTarget, viperMotorsPos);
            liftMotorsSetPower(power);
            // Have we achieved the target?
            // (temporarily limit to 16 cycles when verifying any major math changes!)
//          if( liftMotorCycles >= 16 ) {
            if( ticksToGoAbs <= 10 ) {
                liftMotorsSetPower( 0.0 );
                if( ++liftMotorWait >= waitCycles ) {
                    liftMotorPIDAuto = false;
//                    writeLiftLog();
                }
            }
            // No, still not within tolerance of desired target
            else {
                // Reset the wait count back to zero
                liftMotorWait = 0;
            }
        } // liftMotorPIDAuto
    } // liftPIDPosRun

    // Functions to have the scorer grab the pixels from the storage bins
    public enum PixelGrabActivity {
        IDLE,
        EXTENDING,
        GRABBING
    }
    public PixelGrabActivity pixelGrabState = PixelGrabActivity.IDLE;
    private ElapsedTime pixelGrabTimer = new ElapsedTime();
    public void startPixelGrab()
    {
        // Ensure pre-conditions are met
        if((pixelGrabState == PixelGrabActivity.IDLE) &&
                (pixelScoreState == PixelScoreActivity.IDLE)) {
            pushServo.setPosition(PUSH_SERVO_GRAB);
            pixelGrabTimer.reset();
            pixelGrabState = PixelGrabActivity.EXTENDING;
        }
    }
    public void processPixelGrab()
    {
        switch(pixelGrabState){
            case EXTENDING:
                // We have extended to the desired position
                if(getPushServoAngle() <= PUSH_SERVO_GRAB_ANGLE) {
                    // Rotate both fingers to grab the pixels
                    fingerServo1.setPosition(FINGER1_SERVO_GRAB);
                    fingerServo2.setPosition(FINGER2_SERVO_GRAB);
                    pixelGrabTimer.reset();
                    pixelGrabState = PixelGrabActivity.GRABBING;
                }
                // This is a timeout, do we want to retract and try again or just go idle and allow
                // driver to retry?
                else if(pixelGrabTimer.milliseconds() > 1000.0) {
                    telemetry.addData("processPixelGrab", "Timed out extending");
                    telemetry.addData("processPixelGrab", "Push: %.2f", getPushServoAngle());
                    telemetry.update();
                    telemetrySleep();
                    pushServo.setPosition(PUSH_SERVO_INIT);
                    pixelGrabState = PixelGrabActivity.IDLE;
                }
                break;
            case GRABBING:
                // We have grabbed the pixels
                if((getFingerServo1Angle() <= FINGER1_SERVO_GRAB_ANGLE) &&
                        (getFingerServo2Angle() <= FINGER2_SERVO_GRAB_ANGLE)) {
                    pixelGrabState = PixelGrabActivity.IDLE;
                }
                // This is a timeout, can't think of any other error than angle is wrong? I think for
                // now we proceed as if we grabbed.
                else if(pixelGrabTimer.milliseconds() > 1000.0) {
                    telemetry.addData("processPixelGrab", "Timed out grabbing");
                    telemetry.addData("processPixelGrab", "Finger1: %.2f Finger2: %.2f", getFingerServo1Angle(), getFingerServo2Angle());
                    telemetry.update();
                    telemetrySleep();
                    pixelGrabState = PixelGrabActivity.IDLE;
                }
                break;
            case IDLE:
            default:
                break;
        }
    }

    // Functions to have the scorer score the pixels on the backdrop
    public enum PixelScoreActivity {
        IDLE,
        EXTENDING,
        HOLDING,
        RELEASING,
        RETRACTING,
        ROTATING
    }
    public PixelScoreActivity pixelScoreState = PixelScoreActivity.IDLE;
    private ElapsedTime pixelScoreTimer = new ElapsedTime();
    public void startPixelScore()
    {
        // Ensure pre-conditions are met
        if((pixelScoreState == PixelScoreActivity.IDLE) &&
                (pixelGrabState == PixelGrabActivity.IDLE) &&
                (viperMotorsPos > VIPER_EXTEND_BIN)) {
            // Rotate wrist to the scoring position
            wristServo.setPosition(WRIST_SERVO_DROP);
            // Fully extend the wrist assembly toward the backdrop
            // (we should do these automatically once we're above the top of the bin??)
            pushServo.setPosition(PUSH_SERVO_DROP);
            pixelScoreTimer.reset();
            pixelScoreState = PixelScoreActivity.EXTENDING;
        }
    }
    public void processPixelScore() {
        switch(pixelScoreState){
            case EXTENDING:
                // We have extended to the desired position
                if((getPushServoAngle() <= PUSH_SERVO_DROP_ANGLE) &&
                        (getWristServoAngle() <= WRIST_SERVO_DROP_ANGLE)) {
                    pixelScoreState = PixelScoreActivity.HOLDING;
                }
                // This is a timeout, can't think of any other error than angle is wrong? I think for
                // now we proceed as if we extended.
                else if(pixelScoreTimer.milliseconds() > 1000.0) {
                    telemetry.addData("processPixelScore", "Timed out extending");
                    telemetry.addData("processPixelScore", "Push: %.2f Wrist: %.2f", getPushServoAngle(), getWristServoAngle());
                    // Rotate both fingers to release the pixels
                    telemetry.update();
                    telemetrySleep();
                    pixelScoreState = PixelScoreActivity.HOLDING;
                }
                break;
            case HOLDING:
                // We have extended to the desired position
                if(scorePixelGo) {
                    // Rotate both fingers to release the pixels
                    scorePixelGo = false;
                    fingerServo1.setPosition(FINGER1_SERVO_DROP);
                    fingerServo2.setPosition(FINGER2_SERVO_DROP);
                    pixelScoreTimer.reset();
                    pixelScoreState = PixelScoreActivity.RELEASING;
                }
                break;
            case RELEASING:
                if((getFingerServo1Angle() >= FINGER1_SERVO_DROP_ANGLE) &&
                        (getFingerServo2Angle() >= FINGER2_SERVO_DROP_ANGLE)) {
                    // Pull back to the safe/stored position
                    pushServo.setPosition(PUSH_SERVO_SAFE);
                    pixelScoreTimer.reset();
                    pixelScoreState = PixelScoreActivity.RETRACTING;
                }
                // This is a timeout, can't think of any other error than angle is wrong? I think for
                // now we proceed as if we extended.
                else if(pixelScoreTimer.milliseconds() > 1000.0) {
                    telemetry.addData("processPixelScore", "Timed out releasing");
                    telemetry.addData("processPixelScore", "Finger1: %.2f Finger2: %.2f", getFingerServo1Angle(), getFingerServo2Angle());
                    // Pull back to the safe/stored position
                    telemetry.update();
                    telemetrySleep();
                    pushServo.setPosition(PUSH_SERVO_SAFE);
                    pixelScoreTimer.reset();
                    pixelScoreState = PixelScoreActivity.RETRACTING;
                }
                break;
            case RETRACTING:
                // We have pulled back to where we have cleared the pixels and can rotate the wrist
                // to safe position
                if(getPushServoAngle() >= PUSH_SERVO_PIXEL_CLEAR_ANGLE) {
                    wristServo.setPosition(WRIST_SERVO_GRAB);
                    pixelScoreTimer.reset();
                    pixelScoreState = PixelScoreActivity.ROTATING;
                }
                // This is a timeout, can't think of any other error than angle is wrong? I think for
                // now we proceed as if we extended.
                else if(pixelScoreTimer.milliseconds() > 1000.0) {
                    telemetry.addData("processPixelScore", "Timed out retracting");
                    telemetry.addData("processPixelScore", "Push: %.2f", getPushServoAngle());
                    // Pull back to the safe/stored position
                    telemetry.update();
                    telemetrySleep();
                    wristServo.setPosition(WRIST_SERVO_GRAB);
                    pixelScoreTimer.reset();
                    pixelScoreState = PixelScoreActivity.ROTATING;
                }
                break;
            case ROTATING:
                // We are storing the scorer to safe position for operating the lift 185 184.5
                if((getPushServoAngle() >= PUSH_SERVO_SAFE_ANGLE) &&
                        (getWristServoAngle() >= WRIST_SERVO_GRAB_ANGLE)) {
                    pixelScoreState = PixelScoreActivity.IDLE;
                }
                // This is a timeout, can't think of any other error than angle is wrong? I think for
                // now we proceed as if we extended.
                else if(pixelScoreTimer.milliseconds() > 1000.0) {
                    telemetry.addData("processPixelScore", "Timed out rotating");
                    telemetry.addData("processPixelScore", "Push: %.2f Wrist: %.2f", getPushServoAngle(), getWristServoAngle());
                    telemetry.update();
                    telemetrySleep();
                    pixelScoreState = PixelScoreActivity.IDLE;
                }
                break;
            case IDLE:
            default:
                break;
        }
    }

    // Functions to have the scorer score the pixels on the backdrop during Auto
    public enum PixelScoreAutoActivity {
        IDLE,
        EXTENDING,
        STABILIZING,
        RELEASING,
        RETRACTING,
        RETRACTED
    }
    public PixelScoreAutoActivity pixelScoreAutoState = PixelScoreAutoActivity.IDLE;
    private ElapsedTime pixelScoreAutoTimer = new ElapsedTime();
    public void startPixelScoreAuto()
    {
        // Ensure pre-conditions are met
        if((pixelScoreAutoState == PixelScoreAutoActivity.IDLE) &&
                (pixelGrabState == PixelGrabActivity.IDLE) &&
                (pixelScoreState == PixelScoreActivity.IDLE) &&
                (viperMotorsPos >= (VIPER_EXTEND_AUTO - 30))) {
            // Fully extend the wrist assembly toward the backdrop
            // (we should do these automatically once we're above the top of the bin??)
            pushServo.setPosition(PUSH_SERVO_DROP);
            pixelScoreAutoTimer.reset();
            pixelScoreAutoState = PixelScoreAutoActivity.EXTENDING;
        }
    }
    public void processPixelScoreAuto() {
        switch(pixelScoreAutoState){
            case EXTENDING:
                // We have extended to the desired position
                if(getPushServoAngle() <= PUSH_SERVO_DROP_ANGLE) {
                    pixelScoreAutoTimer.reset();
                    pixelScoreAutoState = PixelScoreAutoActivity.STABILIZING;
                }
                // This is a timeout, can't think of any other error than angle is wrong? I think for
                // now we proceed as if we extended.
                else if(pixelScoreAutoTimer.milliseconds() > 1000.0) {
                    telemetry.addData("processPixelScoreAuto", "Timed out extending");
                    telemetry.addData("processPixelScoreAuto", "Push: %.2f", getPushServoAngle());
                    telemetry.update();
                    telemetrySleep();
                    pixelScoreAutoTimer.reset();
                    pixelScoreAutoState = PixelScoreAutoActivity.STABILIZING;
                }
                break;
            case STABILIZING: // don't fling the pixels against the backdrop
                if(pixelScoreAutoTimer.milliseconds() >= 500) {
                    // Rotate both fingers to release the pixels
                    fingerServo1.setPosition(FINGER1_SERVO_DROP);
                    fingerServo2.setPosition(FINGER2_SERVO_DROP);
                    pixelScoreAutoTimer.reset();
                    pixelScoreAutoState = PixelScoreAutoActivity.RELEASING;
                }
                break;
            case RELEASING:
                // We have extended to the desired position 176 182
                if((getFingerServo1Angle() >= FINGER1_SERVO_DROP_ANGLE) &&
                   (getFingerServo2Angle() >= FINGER2_SERVO_DROP_ANGLE)) {
                    // Pull back to the safe/stored position
                    pushServo.setPosition(PUSH_SERVO_SAFE);
                    pixelScoreAutoTimer.reset();
                    pixelScoreAutoState = PixelScoreAutoActivity.RETRACTING;
                }
                // This is a timeout, can't think of any other error than angle is wrong? I think for
                // now we proceed as if we extended.
                else if(pixelScoreAutoTimer.milliseconds() > 1000.0) {
                    telemetry.addData("processPixelScoreAuto", "Timed out releasing");
                    telemetry.addData("processPixelScoreAuto", "Finger1: %.2f Finger2: %.2f", getFingerServo1Angle(), getFingerServo2Angle());
                    // Pull back to the safe/stored position
                    telemetry.update();
                    telemetrySleep();
                    pushServo.setPosition(PUSH_SERVO_SAFE);
                    pixelScoreAutoTimer.reset();
                    pixelScoreAutoState = PixelScoreAutoActivity.RETRACTING;
                }
                break;
            case RETRACTING:
                // We have pulled back to where we have cleared the pixels and can rotate the wrist
                // to safe position
                if(getPushServoAngle() >= PUSH_SERVO_PIXEL_CLEAR_ANGLE) {
                    wristServo.setPosition(WRIST_SERVO_GRAB);
                    pixelScoreAutoTimer.reset();
                    pixelScoreAutoState = PixelScoreAutoActivity.RETRACTED;
                }
                // This is a timeout, can't think of any other error than angle is wrong? I think for
                // now we proceed as if we extended.
                else if(pixelScoreAutoTimer.milliseconds() > 1000.0) {
                    telemetry.addData("processPixelScoreAuto", "Timed out retracting");
                    telemetry.addData("processPixelScoreAuto", "Push: %.2f", getPushServoAngle());
                    // Pull back to the safe/stored position
                    telemetry.update();
                    telemetrySleep();
                    wristServo.setPosition(WRIST_SERVO_GRAB);  // never rotated, but just to be safe!
                    pixelScoreAutoTimer.reset();
                    pixelScoreAutoState = PixelScoreAutoActivity.RETRACTED;
                }
                break;
            case RETRACTED:
                // We are storing the scorer to safe position for operating the lift 185 184.5
                if((getPushServoAngle() >= PUSH_SERVO_SAFE_ANGLE)) {
                    pixelScoreAutoState = pixelScoreAutoState.IDLE;
                }
                // This is a timeout, can't think of any other error than angle is wrong? I think for
                // now we proceed as if we extended.
                else if(pixelScoreAutoTimer.milliseconds() > 1000.0) {
                    telemetry.addData("processPixelScoreAuto", "Timed out retracted");
                    telemetry.addData("processPixelScoreAuto", "Push: %.2f", getPushServoAngle());
                    telemetry.update();
                    telemetrySleep();
                    pixelScoreAutoState = pixelScoreAutoState.IDLE;
                }
                break;
            case IDLE:
            default:
                break;
        }
    }
    public enum LiftMoveActivity {
        IDLE,
        LIFTING_SAFE,
        LIFTING,
        ROTATING
    }
    public LiftMoveActivity liftMoveState = LiftMoveActivity.IDLE;
    private ElapsedTime liftMoveTimer = new ElapsedTime();
    private int liftMoveTarget;
    public void startLiftMove(int liftTarget)
    {
        // Ensure pre-conditions are met
        if((pixelScoreAutoState == PixelScoreAutoActivity.IDLE) &&
                (pixelGrabState == PixelGrabActivity.IDLE) &&
                (pixelScoreState == PixelScoreActivity.IDLE)) {
            // startViperSlideExtension handles lift power vs lower power
            liftMoveTarget = liftTarget;
            startViperSlideExtension(liftMoveTarget);
            liftMoveTimer.reset();
            liftMoveState = LiftMoveActivity.LIFTING_SAFE;
        }
    }
    public void processLiftMove() {
        switch(liftMoveState){
            // Make sure we are over the bin to rotate
            case LIFTING_SAFE:
                // We are above the pixel bin and can start rotating
                if(viperMotorsPos >= VIPER_EXTEND_BIN) {
                    // This shouldn't happen, but make sure we aren't moving below
                    // the pixel bin height before starting wrist movement. Moving below
                    // pixel bin should use the store activity.
                    if(liftMoveTarget >= VIPER_EXTEND_BIN) {
                        wristServo.setPosition(WRIST_SERVO_DROP);
                    }
                    liftMoveTimer.reset();
                    liftMoveState = LiftMoveActivity.LIFTING;
                }
                // This is a timeout, the lift could not get above the bin, we probably
                // want to stop here.
                else if(liftMoveTimer.milliseconds() > 2000.0) {
                    telemetry.addData("processLiftMove", "Timed out lifting safe");
                    telemetry.addData("processLiftMove", "Lift Target: %d Lift Position: %d",
                            liftMoveTarget, viperMotorsPos);
                    telemetry.update();
                    telemetrySleep();
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
                    telemetry.addData("processLiftMove", "Timed out lifting");
                    telemetry.addData("processLiftMove", "Lift Target: %d Lift Position: %d",
                            liftMoveTarget, viperMotorsPos);
                    telemetry.update();
                    telemetrySleep();
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
                    telemetry.addData("processLiftMove", "Timed out rotating");
                    telemetry.addData("processLiftMove", "Wrist: %.2f", getWristServoAngle());
                    // Pull back to the safe/stored position
                    telemetry.update();
                    telemetrySleep();
                    liftMoveState = LiftMoveActivity.IDLE;
                }
                break;
            case IDLE:
            default:
                break;
        }
    }
    public enum LiftStoreActivity {
        IDLE,
        ROTATING,
        LOWERING
    }
    public LiftStoreActivity liftStoreState = LiftStoreActivity.IDLE;
    private ElapsedTime liftStoreTimer = new ElapsedTime();
    public void startLiftStore()
    {
        // Ensure pre-conditions are met
        if((pixelScoreAutoState == PixelScoreAutoActivity.IDLE) &&
                (pixelGrabState == PixelGrabActivity.IDLE) &&
                (pixelScoreState == PixelScoreActivity.IDLE) &&
                (viperMotorsPos >= VIPER_EXTEND_BIN)) {
            // Rotate wrist to safe position and pull in
            pushServo.setPosition(PUSH_SERVO_SAFE);
            wristServo.setPosition(WRIST_SERVO_INIT);
            // We want the fingers closed, but don't need them close so don't check them.
            fingerServo1.setPosition(FINGER1_SERVO_DROP);
            fingerServo2.setPosition(FINGER2_SERVO_DROP);
            liftStoreTimer.reset();
            liftStoreState = LiftStoreActivity.ROTATING;
        }
    }
    public void processLiftStore() {
        switch(liftStoreState){
            // Make sure the fingers are in a safe position
            case ROTATING:
                if((getPushServoAngle() >= PUSH_SERVO_SAFE_ANGLE) &&
                        (getWristServoAngle() >= WRIST_SERVO_GRAB_ANGLE)) {
                    liftStoreState = LiftStoreActivity.LOWERING;
                    startViperSlideExtension(VIPER_EXTEND_ZERO);
                    liftStoreTimer.reset();
                }
                // This is a timeout, the fingers are not safe, abort.
                else if(liftStoreTimer.milliseconds() > 1500.0) {
                    telemetry.addData("processLiftStore", "Timed out rotating");
                    telemetry.addData("processLiftStore", "Push: %.2f Wrist: %.2f", getPushServoAngle(), getWristServoAngle());
                    telemetry.update();
                    telemetrySleep();
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
                    telemetry.addData("processLiftStore", "Timed out lowering");
                    telemetry.addData("processLiftStore", "Lift Position: %d",
                            viperMotorsPos);
                    telemetry.update();
                    telemetrySleep();
                    liftStoreState = LiftStoreActivity.IDLE;
                }
                break;
            case IDLE:
            default:
                break;
        }
    }
    private void telemetrySleep() {
//        try {
//            sleep(5000);
//        } catch (InterruptedException ex) {
//            Thread.currentThread().interrupt();
//        }
    }
} /* HardwarePixelbot */
