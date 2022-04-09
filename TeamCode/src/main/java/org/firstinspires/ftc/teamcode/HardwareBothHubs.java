package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.MILLIAMPS;

import static java.lang.Thread.sleep;

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareDrivers.MaxSonarI2CXL;

import java.util.Arrays;
import java.util.Date;
import java.util.EnumMap;
import java.util.Locale;
import java.text.SimpleDateFormat;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Map;

/**
 * Hardware class for goBilda robot (15"x15" chassis with 6" Andymark mecanum wheels)
 */
public class HardwareBothHubs
{
    //====== REV CONTROL/EXPANSION HUBS =====
    LynxModule controlHub;
    LynxModule expansionHub;

    // Expansion Hub current readings
//  public double     h2ampsTotalNow, h2ampsTotalMax = 0.0;
//  public double     h2ampsI2cNow,   h2ampsI2cMax   = 0.0;
//  public double     h2ampsGpioNow,  h2ampsGpioMax  = 0.0;
    // Digital input status can be access by PIN NUMBER (as well as by OBJECT)
//  int               HUB_DIGPIN0 = 0; // Digital I/O input pin #0
//  int               HUB_DIGPIN2 = 2; // Digital I/O input pin #2

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

    protected double COUNTS_PER_MOTOR_REV  = 28.0;    // goBilda Yellow Jacket Planetary Gear Motor Encoders
    protected double DRIVE_GEAR_REDUCTION  = 19.203;  // goBilda 19.2:1 (312rpm) gear ratio with 1:1 bevel gear
    protected double MECANUM_SLIPPAGE      = 1.01;    // one wheel revolution doesn't achieve 6" x 3.1415 of travel.
    protected double WHEEL_DIAMETER_INCHES = 6.0;     // For computing circumference
    protected double COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * MECANUM_SLIPPAGE) / (WHEEL_DIAMETER_INCHES * 3.1415);

    //====== DUCK CAROUSEL MOTOR (RUN_USING_ENCODER) =====
    protected DcMotorEx duckMotor     = null;
    public double       duckMotorVel  = 0.0;           // encoder counts per second

    // Instrumentation:  writing to input/output is SLOW, so to avoid impacting loop time as we capture
    // motor performance we store data to memory until the movement is complete, then dump to a file.
    public boolean          duckMotorLogging   = false; // only enable during development!!
    public final static int DUCKMOTORLOG_SIZE  = 128;   // 128 entries = 2+ seconds @ 16msec/60Hz
    protected double[]      duckMotorLogTime   = new double[DUCKMOTORLOG_SIZE];  // msec
    protected double[]      duckMotorLogVel    = new double[DUCKMOTORLOG_SIZE];  // counts/sec
    protected double[]      duckMotorLogPwr    = new double[DUCKMOTORLOG_SIZE];  // Power
    protected double[]      duckMotorLogAmps   = new double[DUCKMOTORLOG_SIZE];  // mAmp
    protected boolean       duckMotorLogEnable = false;
    protected int           duckMotorLogIndex  = 0;
    protected ElapsedTime   duckMotorTimer     = new ElapsedTime();

    //====== CAPPING ARM MOTOR (RUN_USING_ENCODER) =====
    protected DcMotorEx cappingMotor     = null;
    public boolean      cappingMotorAuto = false;      // Automatic go-to-position in progress
    public int          cappingMotorTgt  = 0;          // go-to-position target encoder count
    public int          cappingMotorCycl = 0;          // go-to-position cycle count
    public int          cappingMotorWait = 0;          // go-to-position wait count (truly there! not just passing thru)
    public int          cappingMotorPos  = 0;          // current encoder count
    public double       cappingMotorVel  = 0.0;        // encoder counts per second
    public double       cappingMotorPwr  = 0.0;        // motor power setpoint (-1.0 to +1.0)
    public double       cappingMotorAmps = 0.0;        // current power draw (Amps)
    public boolean      cappingMotorRamp = false;      // motor power setting is ramping down

    // Instrumentation:  writing to input/output is SLOW, so to avoid impacting loop time as we capture
    // motor performance we store data to memory until the movement is complete, then dump to a file.
    public boolean        cappingMotorLogging = false; // only enable during development!!
    public final static int CAPMOTORLOG_SIZE  = 128;   // 128 entries = 2+ seconds @ 16msec/60Hz
    protected double[]      capMotorLogTime   = new double[CAPMOTORLOG_SIZE];  // msec
    protected int[]         capMotorLogPos    = new int[CAPMOTORLOG_SIZE];     // encoder count
    protected double[]      capMotorLogVel    = new double[CAPMOTORLOG_SIZE];  // counts/sec
    protected double[]      capMotorLogPwr    = new double[CAPMOTORLOG_SIZE];  // Power
    protected double[]      capMotorLogAmps   = new double[CAPMOTORLOG_SIZE];  // mAmp
    protected boolean       capMotorLogEnable = false;
    protected int           capMotorLogIndex  = 0;
    protected ElapsedTime   capMotorTimer     = new ElapsedTime();

    public int          CAPPING_ARM_POS_START   = 0;     // 1150rpm motor (145.1 pulse/rev)
    public int          CAPPING_ARM_POS_STORE   = 907;   // transport position (high enough to miss shared-hub post)
    public int          CAPPING_ARM_POS_LIBERTY = 1099;   // status of liberty pose (end duck-autonomous here)
    public int          CAPPING_ARM_POS_VERTICAL= 1406;  // straight up (minimal motor power required)
    public int          CAPPING_ARM_POS_CAP     = 1866;  // hovering above the alliance shipping hub post
    public int          CAPPING_ARM_POS_CUP     = 2764;  // floor level (to grab CUP  Team Shipping Element)
    public int          CAPPING_ARM_POS_GRAB    = 2893;  // floor level (to grab 7592 Team Shipping Element)
    public int          cappingArmPos = CAPPING_ARM_POS_START;

    // CAPPING ARM WRIST SERVO
    public Servo   wristServo = null;
    public static double wristServoAuto = 0.800;
    public double  WRIST_SERVO_INIT    = 0.950;
    public double  WRIST_SERVO_LIBERTY = 0.500;  // status of liberty pose (end duck-autonomous here)
    public double  WRIST_SERVO_GRAB    = 0.455;
    public double  WRIST_SERVO_CUP     = 0.445;
    public double  WRIST_SERVO_STORE   = 0.427;  // updated wrist encoder
    public double  WRIST_SERVO_CAP     = 0.133;

    public Servo   clawServo = null;
    public double  CLAW_SERVO_INIT   = 0.028;
    public double  CLAW_SERVO_CLOSED = CLAW_SERVO_INIT;
    public double  CLAW_SERVO_OPEN   = 0.300;

    //====== FREIGHT ARM MOTOR (RUN_USING_ENCODER) =====
    protected DcMotorEx freightMotor     = null;
    public boolean      freightMotorAuto = false;      // Automatic go-to-position in progress
    public int          freightMotorTgt  = 0;          // go-to-position target encoder count
    public int          freightMotorCycl = 0;          // go-to-position cycle count
    public int          freightMotorWait = 0;          // go-to-position wait count (truly there! not just passing thru)
    public int          freightMotorPos  = 0;          // current encoder count
    public double       freightMotorDiv  = 624.0;      // Divider value for power computation
    public double       freightMotorVel  = 0.0;        // encoder counts per second
    public double       freightMotorPwr  = 0.0;        // motor power setpoint (-1.0 to +1.0)
    public double       freightMotorAmps = 0.0;        // current power draw (Amps)
    public boolean      freightMotorRamp = false;      // motor power setting is ramping down

    // Instrumentation:  writing to input/output is SLOW, so to avoid impacting loop time as we capture
    // motor performance we store data to memory until the movement is complete, then dump to a file.
    public boolean        freightMotorLogging = false; // only enable during development!!
    public final static int FRGMOTORLOG_SIZE  = 128;   // 128 entries = 2+ seconds @ 16msec/60Hz
    protected double[]      frgMotorLogTime   = new double[FRGMOTORLOG_SIZE];  // msec
    protected int[]         frgMotorLogPos    = new int[FRGMOTORLOG_SIZE];     // encoder count
    protected double[]      frgMotorLogVel    = new double[FRGMOTORLOG_SIZE];  // counts/sec
    protected double[]      frgMotorLogPwr    = new double[FRGMOTORLOG_SIZE];  // Power
    protected double[]      frgMotorLogAmps   = new double[FRGMOTORLOG_SIZE];  // mAmp
    protected boolean       frgMotorLogEnable = false;
    protected int           frgMotorLogIndex  = 0;
    protected ElapsedTime   frgMotorTimer     = new ElapsedTime();

    public int          FREIGHT_ARM_POS_COLLECT    = 0;     // Floor level (power-on position)
    public int          FREIGHT_ARM_POS_SAFE       = 60;    // Low enough that it's safe to raise/lower collector arm
    public int          FREIGHT_ARM_POS_SPIN       = 125;   // Raised enough for box to spin clearly
    public int          FREIGHT_ARM_POS_TRANSPORT1 = 232;   // Horizontal transport position
    public int          FREIGHT_ARM_POS_SHARED     = 330;   // Front scoring into shared shipping hub (assumes pretty full)
    public int          FREIGHT_ARM_POS_ROT_TURRET = 855;  // Approaching vertical (auto-rotate as we PASS THRU vertical)
    public int          FREIGHT_ARM_POS_VERTICAL   = 1126;  // Vertical ("up" vs "down" reverse at this point)
    public int          FREIGHT_ARM_POS_EVASIVE    = 1707;  // Point to start rotating after avoiding the hub
    public int          FREIGHT_ARM_POS_HUB_TOP    = 1737;  // For dumping into hub top level
//    public int          FREIGHT_ARM_POS_HUB_TOP    = 1707;  // For dumping into hub top level
    public int          FREIGHT_ARM_POS_HUB_MIDDLE = 1960;  // For dumping into hub middle level
    public int          FREIGHT_ARM_POS_HUB_BOTTOM = 2160;  // For dumping into hub bottom level
    public int          FREIGHT_ARM_POS_MAX        = 2250;  // Maximum safe rotation without hitting field floor
    public int          FREIGHT_ARM_POS_HUB_TOP_AUTO    = FREIGHT_ARM_POS_HUB_TOP    + 15;
    public int          FREIGHT_ARM_POS_HUB_MIDDLE_AUTO = FREIGHT_ARM_POS_HUB_MIDDLE + 0; // 24
    public int          FREIGHT_ARM_POS_HUB_BOTTOM_AUTO = FREIGHT_ARM_POS_HUB_BOTTOM + 25; // 75

    // These values are also freight-arm counts, but represent limits for horizontal motion
    public int          TURRET_RIGHT_WHEEL         = 175;   // High enough to clear the chassis/wheel on the right
    public int          TURRET_RIGHT_MAX           = 1780;  // ... maximum before we hit the other upright
    public int          TURRET_LEFT_COLLECTOR1     = 315;   // High enough to clear the collector arm on the left
    public int          TURRET_LEFT_COLLECTOR2     = 425;   // High enough to interact with capping arm post/upright
    public int          TURRET_LEFT_MAX            = 1780;  // ... maximum before we hit the other upright

    public Servo        turretServo                = null;
//    public double       TURRET_SERVO_INIT          = 0.505; // we init to the position needed to STORE the freight arm
//    public double       TURRET_SERVO_CENTERED      = 0.505;
//    public double       TURRET_SERVO_SHARED_LEFT   = 0.550; // fixed shift LEFT for use on the shared hub
//    public double       TURRET_SERVO_SHARED_RIGHT  = 0.450; // fixed shift RIGHT for use on the shared hub
//    public double       TURRET_SERVO_BLUE_ALLIANCE = 0.770; // fixed shift LEFT  for use on the alliance hub
//    public double       TURRET_SERVO_RED_ALLIANCE  = 0.240; // fixed shift RIGHT for use on the alliance hub
    public int          turretEncoderPos           = 0;     // Turret servo current position
    public int          turretEncStableCts         = 0;     // Turret at set position counts
    public double       turretTargetPos            = 0.0;   // Value servo set to

    // TBD measure these counts
    public enum TurretPosition {
        INIT(0),
        CENTERED(0),
        SHARED_LEFT(-100),
        SHARED_RIGHT(100),
        BLUE_ALLIANCE(-1174),
        RED_ALLIANCE(1011);

        private final int encoderCount;

        TurretPosition(int encoderCount) {
            this.encoderCount = encoderCount;
        }

        public int getEncoderCount() {
            return encoderCount;
        }
    }
    Map<TurretPosition, Double> turretMap = new EnumMap<>(TurretPosition.class);
    public TurretPosition       turretSetPos = TurretPosition.INIT;

    public Servo        boxServo                   = null;
    public double       BOX_SERVO_INIT             = 0.28;  // we init to the TRANSPORT position
    public double       BOX_SERVO_COLLECT          = 0.48;
    public double       BOX_SERVO_STORED           = 0.40;  // less than TRANSPORT so we can't hold TWO
    public double       BOX_SERVO_TRANSPORT        = 0.28;
    public double       BOX_SERVO_DUMP_TOP         = 0.55;
    public double       BOX_SERVO_DUMP_MIDDLE      = 0.65;
    public double       BOX_SERVO_DUMP_BOTTOM      = 0.75;
    public double       BOX_SERVO_DUMP_FRONT       = 0.80;  // ??

    public Servo        linkServo                  = null;
    public double       LINK_SERVO_INIT            = 0.500; // we init to the FULLY STORED position
    public double       LINK_SERVO_STORED          = 0.500;
    public double       LINK_SERVO_RAISED          = 0.500; // in case we want to tweak/optimize
    public double       LINK_SERVO_LOWERED         = 0.265;

    // This motor has the turret encoder attached.
    protected DcMotorEx sweepMotor    = null;

    //====== NAVIGATION DISTANCE SENSORS =====
    private MaxSonarI2CXL sonarRangeL = null;   // Must include MaxSonarI2CXL.java in teamcode folder
    private MaxSonarI2CXL sonarRangeR = null;
    private MaxSonarI2CXL sonarRangeF = null;
    private MaxSonarI2CXL sonarRangeB = null;
//  public ColorSensor freightIdentifier = null;
//  public DistanceSensor freightFinder = null;
    private NormalizedColorSensor freightFinder = null;
    private DistanceSensor freightDetector = null;
    public float[] hsvValues = new float[3];
    public NormalizedRGBA colors;
    public double freightDistance;
    public final static int FREIGHT_DETECTED_THRESHOLD = 10;

    public final static double BARRIER_NESTED_ROBOT_TILT_AUTO = -3.5;
    public final static double BARRIER_NESTED_ROBOT_TILT_TELE = -3.5;
//  public final static double BARRIER_NESTED_ROBOT_TILT_TELE = -2.0;

//  private DistanceSensor tofRangeL  = null;
//  private DistanceSensor tofRangeR  = null;

    private int      sonarRangeLIndex   = 0;                          // 0...4 (SampCnt-1)
    private double[] sonarRangeLSamples = {0,0,0,0,0};                // continuous sampling data (most recent 5)
    private int      sonarRangeLSampCnt = sonarRangeLSamples.length;  // 5
    private double   sonarRangeLMedian  = 0.0;                        // CM (divide by 2.54 for INCHES)
    public  double   sonarRangeLStdev   = 0.0;

    private int      sonarRangeRIndex   = 0;                          // 0...4 (SampCnt-1)
    private double[] sonarRangeRSamples = {0,0,0,0,0};                // continuous sampling data (most recent 5)
    private int      sonarRangeRSampCnt = sonarRangeRSamples.length;  // 5
    private double   sonarRangeRMedian  = 0.0;                        // CM (divide by 2.54 for INCHES)
    public  double   sonarRangeRStdev   = 0.0;

    private int      sonarRangeFIndex   = 0;                          // 0...4 (SampCnt-1)
    private double[] sonarRangeFSamples = {0,0,0,0,0};                // continuous sampling data (most recent 5)
    private int      sonarRangeFSampCnt = sonarRangeLSamples.length;  // 5
    private double   sonarRangeFMedian  = 0.0;                        // CM (divide by 2.54 for INCHES)
    public  double   sonarRangeFStdev   = 0.0;

    private int      sonarRangeBIndex   = 0;                          // 0...4 (SampCnt-1)
    private double[] sonarRangeBSamples = {0,0,0,0,0};                // continuous sampling data (most recent 5)
    private int      sonarRangeBSampCnt = sonarRangeRSamples.length;  // 5
    private double   sonarRangeBMedian  = 0.0;                        // CM (divide by 2.54 for INCHES)
    public  double   sonarRangeBStdev   = 0.0;

//  private int      tofRangeLIndex     = 0;                          // 0...4
//  private double[] tofRangeLSamples   = {0,0,0,0,0};                // results of continuous sampling (most recent 5)
//  private int      tofRangeLSampCnt   = tofRangeLSamples.length;  // 5
//  private double   tofRangeLMedian    = 0.0;
//  public  double   tofRangeLStdev     = 0.0;

//  private int      tofRangeRIndex     = 0;                          // 0...4
//  private double[] tofRangeRSamples   = {0,0,0,0,0};                // results of continuous sampling (most recent 5)
//  private int      tofRangeRSampCnt   = tofRangeRSamples.length;  // 5
//  private double   tofRangeRMedian    = 0.0;
//  public  double   tofRangeRStdev     = 0.0;

    //====== ENCODER RESET FLAG ======
    static private boolean transitionFromAutonomous = false;  // reset 1st time, plus anytime we do teleop-to-teleop

    /* local OpMode members. */
    protected HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareBothHubs(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean isAutonomous ) {
        // Setup the turret map
        // TBD do analysis of blue side position
        turretMap.put(TurretPosition.INIT, 0.505);
        turretMap.put(TurretPosition.CENTERED, 0.505);
        turretMap.put(TurretPosition.SHARED_LEFT, 0.550);
        turretMap.put(TurretPosition.SHARED_RIGHT, 0.450);
        turretMap.put(TurretPosition.BLUE_ALLIANCE, 0.790);
        turretMap.put(TurretPosition.RED_ALLIANCE, 0.240);

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
        frontLeftMotor  = hwMap.get(DcMotorEx.class,"FrontLeft");  // Expansion Hub port 2 (REVERSE)
        frontRightMotor = hwMap.get(DcMotorEx.class,"FrontRight"); // Expansion Hub port 0 (forward)
        rearLeftMotor   = hwMap.get(DcMotorEx.class,"RearLeft");   // Expansion Hub port 3 (REVERSE)
        rearRightMotor  = hwMap.get(DcMotorEx.class,"RearRight");  // Expansion Hub port 1 (forward)

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

        //Initialize duck carousel motor
        duckMotor = hwMap.get(DcMotorEx.class,"DuckMotor");
        duckMotor.setDirection(DcMotor.Direction.REVERSE);  // goBilda fwd/rev opposite of Matrix motors!
        duckMotor.setPower( 0.0 );
        duckMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        duckMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duckMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PIDFCoefficients duckPIDF = new PIDFCoefficients( 10.0,10.0,1.0,1.0, MotorControlAlgorithm.PIDF );
        duckMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, duckPIDF );

        //Initialize capping arm motor
        cappingMotor = hwMap.get(DcMotorEx.class,"CappingMotor");
        cappingMotor.setDirection(DcMotor.Direction.FORWARD);
        cappingMotor.setPower( 0.0 );
        if (!transitionFromAutonomous) {
            cappingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
//      cappingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cappingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        cappingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//      PIDFCoefficients cappingPIDF = new PIDFCoefficients( 10.0,10.0,1.0,1.0, MotorControlAlgorithm.PIDF );
//      cappingMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, cappingPIDF );

        wristServo = hwMap.servo.get("WristServo");    // servo port 0 (hub 2)
        if (!transitionFromAutonomous) {
            wristServo.setPosition( WRIST_SERVO_INIT );
        } else {
            wristServo.setPosition( wristServoAuto );
        }

        clawServo = hwMap.servo.get("ClawServo");      // servo port 1 (hub 2)
        clawServo.setPosition( CLAW_SERVO_INIT );

        //Initialize freight arm motor
        freightMotor = hwMap.get(DcMotorEx.class,"FreightMotor");
        freightMotor.setDirection(DcMotor.Direction.FORWARD);
        freightMotor.setPower( 0.0 );
        if (!transitionFromAutonomous) {
            freightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
//      freightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        freightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        freightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//      PIDFCoefficients freightPIDF = new PIDFCoefficients( 10.0,10.0,1.0,1.0, MotorControlAlgorithm.PIDF );
//      freightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, freightPIDF );

        turretServo = hwMap.servo.get("DT2kServo");      // servo port 3 (hub 2)
        turretPositionSet( TurretPosition.INIT );
        // Don't know of a better way to handle this, but want to make sure the
        // servo is centered before we set the encoder to 0.
        try {
            sleep(500);
        } catch (InterruptedException ioex) {}

        boxServo = hwMap.servo.get("BoxServo");          // servo port 4 (hub 2)
        if (!transitionFromAutonomous) {
            boxServo.setPosition( BOX_SERVO_INIT );
        }

        linkServo = hwMap.servo.get("linkServo");        // servo port 5 (hub 2)
        if (!transitionFromAutonomous) {
            linkServo.setPosition( LINK_SERVO_INIT );
        }

        //Initialize sweeper motor
        //It is important to perform this after setting the turret servo to init
        //so the 0 encoder position matches the init position.
        sweepMotor = hwMap.get(DcMotorEx.class,"sweepMotor");
        sweepMotor.setDirection(DcMotor.Direction.FORWARD);
        sweepMotor.setPower( 0.0 );
        if (!transitionFromAutonomous) {
            sweepMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
//      sweepMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sweepMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sweepMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Instantiate Maxbotics ultrasonic range sensors (sensors wired to I2C ports)
        sonarRangeL = hwMap.get( MaxSonarI2CXL.class, "left_ultrasonic" );
        sonarRangeR = hwMap.get( MaxSonarI2CXL.class, "right_ultrasonic" );
        sonarRangeF = hwMap.get( MaxSonarI2CXL.class, "front_ultrasonic" );
        sonarRangeB = hwMap.get( MaxSonarI2CXL.class, "back_ultrasonic" );
        freightFinder = hwMap.get(NormalizedColorSensor.class, "freight_finder");
        freightDetector = (DistanceSensor)freightFinder;
        freightFinder.setGain(5.0f);

        //Instantiate REV 2-meter Time-Of-Flight Distance Sensors
//      tofRangeL   = hwMap.get( DistanceSensor.class, "ToF_distanceL" );
//      tofRangeR   = hwMap.get( DistanceSensor.class, "ToF_distanceR" );

        // Initialize REV Expansion Hub IMU
        initIMU();

        // Make a note that we just finished autonomous setup
        transitionFromAutonomous = (isAutonomous)? true:false;
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
    public void readBulkData() {
        // For MANUAL mode, we must clear the BulkCache once per control cycle
        expansionHub.clearBulkCache();
        controlHub.clearBulkCache();
        // Get a fresh set of values for this cycle
        //   getCurrentPosition() / getTargetPosition() / getTargetPositionTolerance()
        //   getPower() / getVelocity() / getCurrent()
        frontLeftMotorPos  = frontLeftMotor.getCurrentPosition();
        frontLeftMotorVel  = frontLeftMotor.getVelocity();
        frontLeftMotorAmps = frontLeftMotor.getCurrent(MILLIAMPS);
        frontRightMotorPos = frontRightMotor.getCurrentPosition();
        frontRightMotorVel = frontRightMotor.getVelocity();
        frontRightMotorAmps= frontRightMotor.getCurrent(MILLIAMPS);
        rearRightMotorPos  = rearRightMotor.getCurrentPosition();
        rearRightMotorVel  = rearRightMotor.getVelocity();
        rearRightMotorAmps = rearRightMotor.getCurrent(MILLIAMPS);
        rearLeftMotorPos   = rearLeftMotor.getCurrentPosition();
        rearLeftMotorVel   = rearLeftMotor.getVelocity();
        rearLeftMotorAmps  = rearLeftMotor.getCurrent(MILLIAMPS);
        duckMotorVel       = duckMotor.getVelocity();

        cappingMotorPos    = cappingMotor.getCurrentPosition();
        cappingMotorVel    = cappingMotor.getVelocity();
        cappingMotorAmps   = cappingMotor.getCurrent( MILLIAMPS );
        double cappingMotorPwrPrior = cappingMotorPwr;
        cappingMotorPwr    = cappingMotor.getPower();
        cappingMotorRamp   = isPwrRampingDown( cappingMotorPwrPrior, cappingMotorPwr );

        freightMotorPos    = freightMotor.getCurrentPosition();
        freightMotorVel    = freightMotor.getVelocity();
        freightMotorAmps   = freightMotor.getCurrent( MILLIAMPS );
        double freightMotorPwrPrior = freightMotorPwr;
        freightMotorPwr    = freightMotor.getPower();
        freightMotorRamp   = isPwrRampingDown( freightMotorPwrPrior, freightMotorPwr );

        turretEncoderPos   = sweepMotor.getCurrentPosition();
        if(isTurretAtPosition(turretSetPos)) {
            turretEncStableCts++;
        } else {
            turretEncStableCts = 0;
        }

        // Do we need to capture capping-arm instrumentation data?
        if( capMotorLogEnable ) {
           capMotorLogTime[capMotorLogIndex] = capMotorTimer.milliseconds();
           capMotorLogPos[capMotorLogIndex]  = cappingMotorPos;
           capMotorLogVel[capMotorLogIndex]  = cappingMotorVel;
           capMotorLogPwr[capMotorLogIndex]  = cappingMotorPwr;
           capMotorLogAmps[capMotorLogIndex] = cappingMotorAmps;
           // If the log is now full, disable further logging
           if( ++capMotorLogIndex >= CAPMOTORLOG_SIZE )
               capMotorLogEnable = false;
        } // capMotorLogEnable

        // Do we need to capture freight-arm instrumentation data?
        if( frgMotorLogEnable ) {
           frgMotorLogTime[frgMotorLogIndex] = frgMotorTimer.milliseconds();
           frgMotorLogPos[frgMotorLogIndex]  = freightMotorPos;
           frgMotorLogVel[frgMotorLogIndex]  = freightMotorVel;
           frgMotorLogPwr[frgMotorLogIndex]  = freightMotorPwr;
           frgMotorLogAmps[frgMotorLogIndex] = freightMotorAmps;
           // If the log is now full, disable further logging
           if( ++frgMotorLogIndex >= FRGMOTORLOG_SIZE )
               frgMotorLogEnable = false;
        } // frgMotorLogEnable

        // Do we need to capture duck motor instrumentation data?
        if( duckMotorLogEnable ) {
           duckMotorLogTime[duckMotorLogIndex] = duckMotorTimer.milliseconds();
           duckMotorLogVel[duckMotorLogIndex]  = duckMotorVel;
           duckMotorLogPwr[duckMotorLogIndex]  = duckMotor.getPower();
           duckMotorLogAmps[duckMotorLogIndex] = duckMotor.getCurrent( MILLIAMPS );;
           // If the log is now full, disable further logging
           if( ++duckMotorLogIndex >= DUCKMOTORLOG_SIZE )
               duckMotorLogEnable = false;
        } // duckMotorLogEnable

    } // readBulkData

    /*--------------------------------------------------------------------------------------------*/
    public void writeCappingLog() {
        // Are we even logging these events?
        if( !cappingMotorLogging) return;
        // Movement must be complete (disable further logging to memory)
        capMotorLogEnable = false;
        // Create a subdirectory based on DATE
        String dateString = new SimpleDateFormat("yyyy-MM-dd", Locale.getDefault()).format(new Date());
        String directoryPath = Environment.getExternalStorageDirectory().getPath() + "//FIRST//CappingArm//" + dateString;
        // Ensure that directory path exists
        File directory = new File(directoryPath);
        directory.mkdirs();
        // Create a filename based on TIME
        String timeString = new SimpleDateFormat("hh-mm-ss", Locale.getDefault()).format(new Date());
        String filePath = directoryPath + "/" + "capping_" + timeString + ".txt";
        // Open the file
        FileWriter cappingLog;
        try {
            cappingLog = new FileWriter(filePath, false);
            // Log the current PIDF settings
//          PIDFCoefficients currPIDF = cappingMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//          cappingLog.write("CappingArm " + currPIDF.toString() + "\r\n");
            cappingLog.write("CappingArm\r\n");
            cappingLog.write("Target position," + cappingMotorTgt + "\r\n");
            // Log Column Headings
            cappingLog.write("msec,pwr,mAmp,cts/sec,encoder\r\n");
            // Log all the data recorded
            for( int i=0; i<capMotorLogIndex; i++ ) {
                String msecString = String.format("%.3f, ", capMotorLogTime[i] );
                String pwrString  = String.format("%.3f, ", capMotorLogPwr[i]  );
                String ampString  = String.format("%.0f, ", capMotorLogAmps[i] );
                String velString  = String.format("%.0f, ", capMotorLogVel[i]  );
                String posString  = String.format("%d\r\n", capMotorLogPos[i]  );
                cappingLog.write( msecString + pwrString + ampString + velString + posString );
            }
            cappingLog.flush();
            cappingLog.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    } // writeCappingLog()
    
    /*--------------------------------------------------------------------------------------------*/
    public void writeFreightLog() {
        // Are we even logging these events?
        if( !freightMotorLogging) return;
        // Movement must be complete (disable further logging to memory)
        frgMotorLogEnable = false;
        // Create a subdirectory based on DATE
        String dateString = new SimpleDateFormat("yyyy-MM-dd", Locale.getDefault()).format(new Date());
        String directoryPath = Environment.getExternalStorageDirectory().getPath() + "//FIRST//FreightArm//" + dateString;
        // Ensure that directory path exists
        File directory = new File(directoryPath);
        directory.mkdirs();
        // Create a filename based on TIME
        String timeString = new SimpleDateFormat("hh-mm-ss", Locale.getDefault()).format(new Date());
        String filePath = directoryPath + "/" + "freight_" + timeString + ".txt";
        // Open the file
        FileWriter freightLog;
        try {
            freightLog = new FileWriter(filePath, false);
            // Log the current PIDF settings
//          PIDFCoefficients currPIDF = freightMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//          freightLog.write("FreightArm " + currPIDF.toString() + "\r\n");
            freightLog.write("FreightArm\r\n");
            freightLog.write("Target position," + freightMotorTgt + "\r\n");
            // Log Column Headings
            freightLog.write("msec,pwr,mAmp,cts/sec,encoder\r\n");
            // Log all the data recorded
            for( int i=0; i<frgMotorLogIndex; i++ ) {
                String msecString = String.format("%.3f, ", frgMotorLogTime[i] );
                String pwrString  = String.format("%.3f, ", frgMotorLogPwr[i]  );
                String ampString  = String.format("%.0f, ", frgMotorLogAmps[i] );
                String velString  = String.format("%.0f, ", frgMotorLogVel[i]  );
                String posString  = String.format("%d\r\n", frgMotorLogPos[i]  );
                freightLog.write( msecString + pwrString + ampString + velString + posString );
            }
            freightLog.flush();
            freightLog.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    } // writeFreightLog()
    
    /*--------------------------------------------------------------------------------------------*/
    public void writeDuckLog() {
        // Are we even logging these events?
        if( !duckMotorLogging) return;
        // If we didn't fill the buffer, then operator stopped the motor (disable further logging to memory)
        duckMotorLogEnable = false;
        // Create a subdirectory based on DATE
        String dateString = new SimpleDateFormat("yyyy-MM-dd", Locale.getDefault()).format(new Date());
        String directoryPath = Environment.getExternalStorageDirectory().getPath() + "//FIRST//DuckMotor//" + dateString;
        // Ensure that directory path exists
        File directory = new File(directoryPath);
        directory.mkdirs();
        // Create a filename based on TIME
        String timeString = new SimpleDateFormat("hh-mm-ss", Locale.getDefault()).format(new Date());
        String filePath = directoryPath + "/" + "duck_" + timeString + ".txt";
        // Open the file
        FileWriter duckLog;
        try {
            duckLog = new FileWriter(filePath, false);
            // Log the current PIDF settings
            PIDFCoefficients currPIDF = duckMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            duckLog.write("DuckMotor " + currPIDF.toString() + "\r\n");
            // Log Column Headings
            duckLog.write("msec,mAmp,pwr,cts/sec\r\n");
            // Log all the data recorded
            for( int i=0; i<duckMotorLogIndex; i++ ) {
                String msecString = String.format("%.3f, ", duckMotorLogTime[i] );
                String ampString  = String.format("%.0f, ", duckMotorLogAmps[i] );
                String pwrString  = String.format("%.3f, ", duckMotorLogPwr[i]  );
                String velString  = String.format("%.0f\r\n", duckMotorLogVel[i]);
                duckLog.write( msecString + ampString + pwrString + velString );
            }
            duckLog.flush();
            duckLog.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    } // writeDuckLog()
    
    /*--------------------------------------------------------------------------------------------*/
    public void driveTrainMotors( double frontLeft, double frontRight, double rearLeft, double rearRight )
    {
        frontLeftMotor.setPower( frontLeft );
        frontRightMotor.setPower( frontRight );
        rearLeftMotor.setPower( rearLeft );
        rearRightMotor.setPower( rearRight );
    } // driveTrainMotors

    /*--------------------------------------------------------------------------------------------*/
    public void driveTrainMotorsZero()
    {
        frontLeftMotor.setPower( 0.0 );
        frontRightMotor.setPower( 0.0 );
        rearLeftMotor.setPower( 0.0 );
        rearRightMotor.setPower( 0.0 );
    } // driveTrainMotorsZero

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
    /* cappingArmPosition()                                                                       */
    /* - newPos = desired new arm position                                                        */
    /* - motorPower = desired motor power level to get there                                      */
    public void cappingArmPosition( int newPos, double motorPower )
    {
        // Are we ALREADY at the specified position?
        if( Math.abs(newPos-cappingMotorPos) < 20 )
           return;

        // Update the target position
        cappingMotorTgt = newPos;

        // Ensure motor is stopped/stationary and abort any prior RUN_TO_POSITION command
        cappingMotor.setPower( 0.0 );
        cappingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Configure new target encoder count
        cappingMotor.setTargetPosition( cappingMotorTgt );

        // Enable RUN_TO_POSITION mode
        cappingMotor.setMode(  DcMotor.RunMode.RUN_TO_POSITION );

        // If logging instrumentation, begin a new dataset now:
        if(cappingMotorLogging) {
            capMotorLogIndex  = 0;
            capMotorLogEnable = true;
            capMotorTimer.reset();
        }

        // Initiate motor movement to the new position
        cappingMotor.setPower( motorPower );

    } // cappingArmPosition

    /*--------------------------------------------------------------------------------------------*/
    /* cappingArmPosInit()                                                                        */
    /* - newPos = desired new arm position                                                        */
    public void cappingArmPosInit( int newPos )
    {
        // Current distance from target (number of encoder counts)
        int ticksToGo = newPos - cappingMotorPos;

        // Are we ALREADY at the specified position?
        if( Math.abs(ticksToGo) < 10 )
           return;

        // Ensure motor is stopped/stationary (aborts any prior unfinished automatic movement)
        cappingMotor.setPower( 0.0 );

        // Establish a new target position & reset counters
        cappingMotorAuto = true;
        cappingMotorTgt  = newPos;
        cappingMotorCycl = 0;
        cappingMotorWait = 0;

        // If logging instrumentation, begin a new dataset now:
        if(cappingMotorLogging) {
            capMotorLogIndex  = 0;
            capMotorLogEnable = true;
            capMotorTimer.reset();
        }

    } // cappingArmPosInit

    /*--------------------------------------------------------------------------------------------*/
    /* cappingArmPosRun()                                                                         */
    public void cappingArmPosRun()
    {
        // Has an automatic movement been initiated?
        if( cappingMotorAuto ) {
          // Keep track of how long we've been doing this
          cappingMotorCycl++;
          // Current distance from target (number of encoder counts)
          int ticksToGo = cappingMotorTgt - cappingMotorPos;
          // Have we achieved the target?
          if( Math.abs(ticksToGo) < 10 ) {
            cappingMotor.setPower( 0.0 );
            if( ++cappingMotorWait >= 5 ) {
                cappingMotorAuto = false;
                writeCappingLog();                
            }
          }
          // No, still not within tolerance of desired target
          else {
              double minPower, maxPower, cappingMotorPower;
              // Reset the wait count back to zero
              cappingMotorWait = 0;
                // Determine our max power (don't go straight to 100% power on start-up)
              switch( cappingMotorCycl ) {
                case 1  : maxPower=0.33; break;
                case 2  : maxPower=0.66; break;
                default : maxPower=1.00;
                }
                // Determine our min power:
                // - Current ramping down implies motor/arm is coming to a stop (allow low power)
                // - Current at zero or increasing means arm won't move unless given enough power
                minPower = (cappingMotorRamp)? 0.01 : 0.30;
              // Compute motor power (automatically reduce as we approach target)
              cappingMotorPower = ticksToGo / 416.0;  // 1620rpm = 103.8 counts per shaft revolution
              cappingMotorPower = Math.copySign( Math.min(Math.abs(cappingMotorPower), maxPower), cappingMotorPower );
              cappingMotorPower = Math.copySign( Math.max(Math.abs(cappingMotorPower), minPower), cappingMotorPower );
              cappingMotor.setPower( cappingMotorPower );
          }
        } // cappingMotorAuto
    } // cappingArmPosRun

    /*--------------------------------------------------------------------------------------------*/
    /* wristPositionAuto()                                                                        */
    /* - newPos     = desired new wrist position                                                  */
    public void wristPositionAuto( double newPos ) {
        wristServo.setPosition( newPos );
        wristServoAuto = newPos;
    } // wristPositionAuto

    /*--------------------------------------------------------------------------------------------*/
    /* freightArmPosition()                                                                       */
    /* - newPos     = desired new arm position                                                    */
    /* - motorPower = desired motor power level to get there                                      */
    public void freightArmPosition( int newPos, double motorPower )
    {
        // Are we ALREADY at the specified position?
        if( Math.abs(newPos-freightMotorPos) < 20 )
            return;

        // Update the target position
        freightMotorTgt = newPos;

        // Ensure motor is stopped/stationary and abort any prior RUN_TO_POSITION command
        freightMotor.setPower( 0.0 );
        freightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Configure new target encoder count
        freightMotor.setTargetPosition( freightMotorTgt );

        // Enable RUN_TO_POSITION mode
        freightMotor.setMode(  DcMotor.RunMode.RUN_TO_POSITION );

        // If logging instrumentation, begin a new dataset now:
        if(freightMotorLogging) {
            frgMotorLogIndex  = 0;
            frgMotorLogEnable = true;
            frgMotorTimer.reset();
        }

        // Initiate motor movement to the new position
        freightMotor.setPower( motorPower );

    } // freightArmPosition

    /*--------------------------------------------------------------------------------------------*/
    /* freightArmPosInit()                                                                        */
    /* - newPos = desired new arm position                                                        */
    public void freightArmPosInit( int newPos )
    {
        // Current distance from target (number of encoder counts)
        int ticksToGo = newPos - freightMotorPos;

        // Are we ALREADY at the specified position?
        if( Math.abs(ticksToGo) < 10 )
           return;

        // Ensure motor is stopped/stationary (aborts any prior unfinished automatic movement)
        freightMotor.setPower( 0.0 );

        // The initial lift from COLLECT to TRANSPORT needs more power to lift against what might
        // be the pile of freight in the warehouse (and a smaller proportional-control divider
        // produces a higher motor power setting)
        if( (newPos == FREIGHT_ARM_POS_TRANSPORT1) && (freightMotorPos < FREIGHT_ARM_POS_TRANSPORT1) )
           freightMotorDiv = 208.0;
        // going from TRANSPORT1 to SHARED_HUB is another short-distance lift
        else if( (newPos == FREIGHT_ARM_POS_SHARED) && (freightMotorPos < FREIGHT_ARM_POS_SHARED) )
            freightMotorDiv = 208.0;
        else // reset back to default proportional-control divider
           freightMotorDiv = 624.0;

        // Establish a new target position & reset counters
        freightMotorAuto = true;
        freightMotorTgt  = newPos;
        freightMotorCycl = 0;
        freightMotorWait = 0;

        // If logging instrumentation, begin a new dataset now:
        if(freightMotorLogging) {
            frgMotorLogIndex  = 0;
            frgMotorLogEnable = true;
            frgMotorTimer.reset();
        }

    } // freightArmPosInit

    /*--------------------------------------------------------------------------------------------*/
    /* freightArmPosRun()                                                                         */
    public void freightArmPosRun()
    {
        // Has an automatic movement been initiated?
        if( freightMotorAuto ) {
            // Keep track of how long we've been doing this
            freightMotorCycl++;
            // Current distance from target (number of encoder counts)
            int ticksToGo = freightMotorTgt - freightMotorPos;
            // Have we achieved the target?
            if( Math.abs(ticksToGo) < 10 ) {
                freightMotor.setPower( 0.0 );
                if( ++freightMotorWait >= 5 ) {
                    freightMotorAuto = false;
                    writeFreightLog();
                }
            }
            // No, still not within tolerance of desired target
            else {
                double minPower, maxPower, freightMotorPower;
                // Reset the wait count back to zero
                freightMotorWait = 0;
                // Determine our max power (don't go straight to 100% power on start-up)
                switch( freightMotorCycl ) {
                case 1  : maxPower=0.33; break;
                case 2  : maxPower=0.66; break;
                default : maxPower=1.00;
                }
                // Determine our min power:
                // - Current ramping down implies motor/arm is coming to a stop (allow low power)
                // - Current at zero or increasing means arm won't move unless given enough power
                minPower = (freightMotorRamp)? 0.35 : 0.40;
                // Compute motor power (automatically reduce as we approach target)
                freightMotorPower = ticksToGo / freightMotorDiv;  // 1620rpm = 103.8 counts per shaft revolution
                freightMotorPower = Math.copySign( Math.min(Math.abs(freightMotorPower), maxPower), freightMotorPower );
                freightMotorPower = Math.copySign( Math.max(Math.abs(freightMotorPower), minPower), freightMotorPower );
                freightMotor.setPower( freightMotorPower );
            }
        } // freightMotorAuto
    } // freightArmPosRun

    /*--------------------------------------------------------------------------------------------*/
    /* turretPositionSet()                                                                        */
    /* - target_position = the target position to command the turret servo to go to                            */
    public void turretPositionSet( TurretPosition target_position ) {
        if((turretSetPos != target_position) || (!isTurretAtPosition(target_position))) {
            turretSetPos = target_position;
            turretEncStableCts = 0;
            turretTargetPos = turretMap.get(turretSetPos);
            turretServo.setPosition(turretTargetPos);
        }
    } // turretPositionSet

    /*--------------------------------------------------------------------------------------------*/
    /* turretPositionShift()                                                                      */
    /* - turret_increment = how much to adjust the turret servo target position                   */
    public void turretPositionShift( double turret_increment ) {
       // ensure we don't go below 0.000
       if( (turretTargetPos + turret_increment) < 0.000 ) {
          // ignore request (can't exceed servo hardware limits)
       }
       else if( (turretTargetPos + turret_increment) > 1.000 ) {
          // ignore request (can't exceed servo hardware limits)
       }
       else {
           turretTargetPos += turret_increment;
          turretServo.setPosition(turretTargetPos);
       }
    } // turretPositionShift

    /*--------------------------------------------------------------------------------------------*/
    /* NOTE ABOUT RANGE SENSORS:                                                                  */
    /* The REV 2m Range Sensor is really only 1.2m (47.2") maximum in DEFAULT mode. Depending on  */        
    /* reflectivity of the surface encountered, it can be even shorter.  For example, the black   */
    /* metal paint on the field wall is highly absorptive, so we only get reliable range readings */
    /* out to 12" or so.  In contrast, the Maxbotics ultrasonic range sensors have a minimum      */
    /* range of 20 cm (about 8").  A combined Autonomous solution that requires both short (< 8") */
    /* and long (> 12-47") requires *both* REV Time-of-Flight (tof) range sensors and Maxbotics   */
    /* Ultrasonic range sensors. Also note that if you mount either ToF/Ultrasonic sensor too low */
    /* on the robot you'll get invalid distance readings due to reflections off the field tiles   */
    /* due to "fanout" of both laser/ultrasonic signals the further you get from the robot.       */
    /*--------------------------------------------------------------------------------------------*/

    // ULTRASONIC READINGS: The ultrasonic driver can be queried in two different update modes:
    // a) getDistanceSync()  sends a new ping and WAITS 50msec for the return
    // b) getDistanceAsync() sends a new ping and RETURNS IMMEDIATELY with the most recent value

    public double updateSonarRangeL() {
        // Query the current range sensor reading as the next sample to our LEFT range dataset
//      sonarRangeLSamples[ sonarRangeLIndex ] = sonarRangeL.getDistanceSync();
        sonarRangeLSamples[ sonarRangeLIndex ] = sonarRangeL.getDistanceAsync();
        if( ++sonarRangeLIndex >= sonarRangeLSampCnt ) sonarRangeLIndex = 0;
        // Create a duplicate copy that's sorted
        double[] sonarRangeLSorted = sonarRangeLSamples;
        Arrays.sort(sonarRangeLSorted);
        // Determine the running median (middle value of the last 5; assumes sonarRangeLSampCnt=5)
        sonarRangeLMedian = sonarRangeLSorted[2];
        // Compute the standard deviation of the collection of readings
        sonarRangeLStdev = stdevSonarRangeL();
        return sonarRangeLMedian;
    } // updateSonarRangeL

    private double stdevSonarRangeL(){
        double sum1=0.0, sum2=0.0, mean;
        for( int i=0; i<sonarRangeLSampCnt; i++ ) {
            sum1 += sonarRangeLSamples[i];
        }
        mean = sum1 / (double)sonarRangeLSampCnt;
        for( int i=0; i<sonarRangeLSampCnt; i++ ) {
            sum2 += Math.pow( (sonarRangeLSamples[i] - mean), 2.0);
        }
        return Math.sqrt( sum2 / (double)sonarRangeLSampCnt );
    } // stdevSonarRangeL

    /*--------------------------------------------------------------------------------------------*/
    public double updateSonarRangeR() {
        // Query the current range sensor reading as the next sample to our RIGHT range dataset
//      sonarRangeRSamples[ sonarRangeRIndex ] = sonarRangeR.getDistanceSync();
        sonarRangeRSamples[ sonarRangeRIndex ] = sonarRangeR.getDistanceAsync();
        if( ++sonarRangeRIndex >= sonarRangeRSampCnt ) sonarRangeRIndex = 0;
        // Create a duplicate copy that's sorted
        double[] sonarRangeRSorted = sonarRangeRSamples;
        Arrays.sort(sonarRangeRSorted);
        // Determine the running median (middle value of the last 5; assumes sonarRangeRSampCnt=5)
        sonarRangeRMedian = sonarRangeRSorted[2];
        // Compute the standard deviation of the collection of readings
        sonarRangeRStdev = stdevSonarRangeR();
        return sonarRangeRMedian;
    } // updateSonarRangeR

    private double stdevSonarRangeR(){
        double sum1=0.0, sum2=0.0, mean;
        for( int i=0; i<sonarRangeRSampCnt; i++ ) {
            sum1 += sonarRangeRSamples[i];
        }
        mean = sum1 / (double)sonarRangeRSampCnt;
        for( int i=0; i<sonarRangeRSampCnt; i++ ) {
            sum2 += Math.pow( (sonarRangeRSamples[i] - mean), 2.0);
        }
        return Math.sqrt( sum2 / (double)sonarRangeRSampCnt );
    } // stdevSonarRangeR
    
    public double updateSonarRangeF() {
        // Query the current range sensor reading as the next sample to our FRONT range dataset
//      sonarRangeFSamples[ sonarRangeFIndex ] = sonarRangeF.getDistanceSync();
        sonarRangeFSamples[ sonarRangeFIndex ] = sonarRangeF.getDistanceAsync();
        if( ++sonarRangeFIndex >= sonarRangeFSampCnt ) sonarRangeFIndex = 0;
        // Create a duplicate copy that's sorted
        double[] sonarRangeFSorted = sonarRangeFSamples;
        Arrays.sort(sonarRangeFSorted);
        // Determine the running median (middle value of the last 5; assumes sonarRangeFSampCnt=5)
        sonarRangeFMedian = sonarRangeFSorted[2];
        // Compute the standard deviation of the collection of readings
        sonarRangeFStdev = stdevSonarRangeF();
        return sonarRangeFMedian;
    } // updateSonarRangeF

    private double stdevSonarRangeF(){
        double sum1=0.0, sum2=0.0, mean;
        for( int i=0; i<sonarRangeFSampCnt; i++ ) {
            sum1 += sonarRangeFSamples[i];
        }
        mean = sum1 / (double)sonarRangeFSampCnt;
        for( int i=0; i<sonarRangeFSampCnt; i++ ) {
            sum2 += Math.pow( (sonarRangeFSamples[i] - mean), 2.0);
        }
        return Math.sqrt( sum2 / (double)sonarRangeFSampCnt );
    } // stdevSonarRangeF

    /*--------------------------------------------------------------------------------------------*/
    public double singleSonarRangeB() {
        // Query the current range sensor reading and wait for a response
        return sonarRangeB.getDistanceSync();
    } // singleSonarRangeB

    public double singleSonarRangeL() {
        // Query the current range sensor reading and wait for a response
        return sonarRangeL.getDistanceSync();
    } // singleSonarRangeL

    public double singleSonarRangeR() {
        // Query the current range sensor reading and wait for a response
        return sonarRangeR.getDistanceSync();
    } // singleSonarRangeR

    public double updateSonarRangeB() {
        // Query the current range sensor reading as the next sample to our BACK range dataset
//      sonarRangeBSamples[ sonarRangeBIndex ] = sonarRangeB.getDistanceSync();
        sonarRangeBSamples[ sonarRangeBIndex ] = sonarRangeB.getDistanceAsync();
        if( ++sonarRangeBIndex >= sonarRangeBSampCnt ) sonarRangeBIndex = 0;
        // Create a duplicate copy that's sorted
        double[] sonarRangeBSorted = sonarRangeBSamples;
        Arrays.sort(sonarRangeBSorted);
        // Determine the running median (middle value of the last 5; assumes sonarRangeBSampCnt=5)
        sonarRangeBMedian = sonarRangeBSorted[2];
        // Compute the standard deviation of the collection of readings
        sonarRangeBStdev = stdevSonarRangeB();
        return sonarRangeBMedian;
    } // updateSonarRangeB

    private double stdevSonarRangeB(){
        double sum1=0.0, sum2=0.0, mean;
        for( int i=0; i<sonarRangeBSampCnt; i++ ) {
            sum1 += sonarRangeBSamples[i];
        }
        mean = sum1 / (double)sonarRangeBSampCnt;
        for( int i=0; i<sonarRangeBSampCnt; i++ ) {
            sum2 += Math.pow( (sonarRangeBSamples[i] - mean), 2.0);
        }
        return Math.sqrt( sum2 / (double)sonarRangeBSampCnt );
    } // stdevSonarRangeB

    public boolean freightPresent() {
        freightDistance = freightDetector.getDistance(DistanceUnit.MM);
        return freightDistance < 64;
    }

    public boolean freightIsCube() {
        colors = freightFinder.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        return hsvValues[1] > 0.5;
    }

    public void stopMotion() {
        // Stop all motion;
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
    }

    protected boolean isTurretAtPosition(TurretPosition position) {
        int turretError = position.getEncoderCount() - turretEncoderPos;

        // This number needs refinement
        // Using +-3 degrees allowed error
        return (Math.abs(turretError) < 68);
    }

    public boolean isTurretStable(TurretPosition position) {
        // This number needs refinement
        return ((turretSetPos == position) && (turretEncStableCts > 1));
    }

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

} /* HardwareBothHubs */
