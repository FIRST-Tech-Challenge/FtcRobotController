package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Thread.sleep;

import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.HardwareDrivers.MaxSonarI2CXL;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.Locale;

/*
 * Hardware class for goBilda robot (12"x15" chassis with 96mm/3.8" goBilda mecanum wheels)
 */
public class HardwareSlimbot
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
    protected double COUNTS_PER_INCH2      = 1738.4;  // 8192 counts-per-rev / (1.5" omni wheel * PI)

    //====== MOTORS FOR GAMEPLAY MECHANISMS (turret / lift) =====
    protected DcMotorEx turretMotor        = null;    // A pair of motors operated as one with a Y cable
    public int          turretMotorPos     = 0;       // current encoder count
    public double       turretMotorVel     = 0.0;     // encoder counts per second
//  public boolean      turretMotorAuto    = false;   // Automatic movement in progress
    public boolean      turretMotorPIDAuto = false;   // Automatic movement in progress (PID)
    public boolean      turretMotorRunning = false;   // We did a turret set power, check the angle
    public int          turretMotorCycles  = 0;       // Automatic movement cycle count
    public int          turretMotorWait    = 0;       // Automatic movement wait count (truly there! not just passing thru)
    public double       turretMotorPwrSet  = 0.0;     // What we commanded the turret power to
    public double       turretMotorPwr     = 0.0;     // What we read-back for turret power (-1.0 to +1.0)
    public double       turretMotorAmps    = 0.0;     // turret motor current power draw (Amps)

    PIDControllerTurret turretPidController;          // PID parameters for the turret motor:
    public double       turretMotorPID_p   = 0.0210;  //   p = proportional
    public double       turretMotorPID_i   = 0.0000;  //   i = integral
    public double       turretMotorPID_d   = 0.0006;  //   d = derivative

    protected AnalogInput turretEncoder    = null;    // US Digital absolute magnetic encoder (MA3)
    public double       turretAngle        = 0.0;     // 0V = 0 degrees; 3.3V = 359.99 degrees
    public double       turretAngleOffset  = 299.0;   // allows us to adjust the 0-360 deg range
    public double       turretAngleTarget  = 0.0;     // Automatic movement target angle (degrees)

    public double       TURRET_ANGLE_MAX       =  170.0;  // absolute encoder angles at maximum rotation RIGHT
    public double       TURRET_ANGLE_AUTO_R    =  116.0;  // scoring the autonomous pre-load cone
    public double       TURRET_ANGLE_CYCLE_R   =  133.5;
    public double       TURRET_ANGLE_5STACK_L  =  118.0;
    public double       TURRET_ANGLE_COLLECT_R =   21.9;
    public double       TURRET_ANGLE_CENTER    =    0.0;  // turret centered
    public double       TURRET_ANGLE_COLLECT_L =  -21.9;
    public double       TURRET_ANGLE_5STACK_R  = -103.0;
    public double       TURRET_ANGLE_AUTO_L    = -131.0;  // scoring the autonomous pre-load cone
    public double       TURRET_ANGLE_CYCLE_L   = -133.5;
    public double       TURRET_ANGLE_AUTO_M_L  = -141.0;  // scoring the autonomous pre-load cone
    public double       TURRET_ANGLE_MIN       = -170.0;  // absolute encoder angles at maximum rotation LEFT

    // Instrumentation:  writing to input/output is SLOW, so to avoid impacting loop time as we capture
    // motor performance we store data to memory until the movement is complete, then dump to a file.
    public boolean          turretMotorLogging   = false; // only enable during development!!
    public final static int TURRETMOTORLOG_SIZE  = 256;   // 256 entries = 2.5 seconds @ 10msec/100Hz
    protected double[]      turretMotorLogTime   = new double[TURRETMOTORLOG_SIZE];  // msec
    protected double[]      turretMotorLogAngle  = new double[TURRETMOTORLOG_SIZE];  // Angle [degrees]
    protected double[]      turretMotorLogPwr    = new double[TURRETMOTORLOG_SIZE];  // Power
    protected double[]      turretMotorLogAmps   = new double[TURRETMOTORLOG_SIZE];  // mAmp
    protected double        turretMotorLogVbat   = 0.0;   // battery voltage at start of movement [mV]
    protected boolean       turretMotorLogEnable = false;
    protected int           turretMotorLogIndex  = 0;
    protected ElapsedTime   turretMotorTimer     = new ElapsedTime();

    protected DcMotorEx liftMotorF         = null;    // FRONT lift motor
    protected DcMotorEx liftMotorB         = null;    // BACK lift motor
//  public boolean      liftMotorAuto      = false;   // Automatic movement in progress
    public boolean      liftMotorPIDAuto   = false;   // Automatic movement in progress (PID)
    public int          liftMotorCycles    = 0;       // Automatic movement cycle count
    public int          liftMotorWait      = 0;       // Automatic movement wait count (truly there! not just passing thru)
    public double       liftMotorPwrSet    = 0.0;     // What we commanded the lift power to
    public double       liftMotorPwr       = 0.0;     // What we read-back for lift power (-1.0 to +1.0)
    public double       liftMotorAmps      = 0.0;     // lift motors current power draw (Amps)
    public boolean      liftMotorRamp      = false;   // motor power setting is ramping down

    PIDControllerWormArm liftPidController;           // PID parameters for the lift motors:
    
    public double        liftMotorPIDr_p   = -0.100;  //  Raise p = proportional 
    public double        liftMotorPIDr_i   =  0.000;  //  Raise i = integral
    public double        liftMotorPIDr_d   = -0.007;  //  Raise d = derivative
    public double        liftMotorPIDr_f   =  0.007;  //  Raise f = sin() function
    
    public double        liftMotorPIDl_p   = -0.030;  //  Lower p = proportional
    public double        liftMotorPIDl_i   =  0.000;  //  Lower i = integral
    public double        liftMotorPIDl_d   =  0.000;  //  Lower d = derivative
    public double        liftMotorPIDl_f   =  0.007;  //  Lower f = sin() function

    public final double LIFT_MOTOR_MAX     =  1.00;   // maximum motor power we allow for the lift (gear slippage!)

    protected AnalogInput liftEncoder      = null;    // US Digital absolute magnetic encoder (MA3)
    public double       liftAngle          = 0.0;     // 0V = 0 degrees; 3.3V = 359.99 degrees
    public double       liftAngleOffset    = 77.2;    // allows us to adjust the -180 to +180 deg range
    public double       liftAngleTarget    = 0.0;     // Automatic movement target angle (degrees)li

    // NOTE: the motor doesn't stop immediately, so set the limits short of the absolute maximum
    // Cone collect:  lift=116.7,  grabber=0.08/0.92, trigger BOTTOM proximity only (existing 3" high team element)
    public double       LIFT_ANGLE_MAX     = 114.0;   // absolute encoder angle at maximum rotation FRONT
    public double       LIFT_ANGLE_ASTART  = 113.0;   // lift position for starting autonomous
    public double       LIFT_ANGLE_COLLECT = 109.0;   // lift position for collecting cones (185mm)
    public double       LIFT_ANGLE_GROUND  = 109.0;   // lift position for GROUND junction
    public double       LIFT_ANGLE_CONE    = 103.0;   // lift position for cleaning a cone on a ground junction
    public double       LIFT_ANGLE_LOW     =  86.0;   // lift position for LOW junction
    public double       LIFT_ANGLE_MOTORS  =  91.0;   // lift position for cleaning front turret motor
    public double       LIFT_ANGLE_5STACK  =  87.0;   // lift position for 5-stack ultrasonic reading
    public double       LIFT_ANGLE_MED     =  57.5;   // lift position for MEDIUM junction (FRONT Teleop)
    public double       LIFT_ANGLE_MED_A   =  50.5;   // lift position for MED junction (FRONT Auto)
    public double       LIFT_ANGLE_HIGH_A  =  34.7;   // lift position for HIGH junction (FRONT Auto)
    public double       LIFT_ANGLE_HIGH    =  24.9;   // lift position for HIGH junction (FRONT Teleop)
    public double       LIFT_ANGLE_HIGH_BA = -41.0;   // lift position for HIGH junction (BACK Auto)
                                                      // (cone is loaded lower for auto, so higher lift point)
    public double       LIFT_ANGLE_HIGH_B  = -45.8;   // lift position for HIGH junction (BACK Teleop)
    public double       LIFT_ANGLE_MED_B   = -73.9;   // lift position for MEDIUM junction (BACK Teleop)
    public double       LIFT_ANGLE_MIN     = -75.0;   //* absolute encoder angle at maximum rotation REAR

    public double[]     coneStackHeights   = {109.0,  // 0 = ground level of 5-stack
                                              108.0,  // 1 = 2nd cone
                                              107.0,  // 2 = 3rd cone
                                              106.0,  // 3 = 4th cone
                                              103.0}; // 4 = 5th cone

    // Instrumentation:  writing to input/output is SLOW, so to avoid impacting loop time as we capture
    // motor performance we store data to memory until the movement is complete, then dump to a file.
    public boolean          liftMotorLogging   = false; // only enable during development!! (RVS)
    public final static int LIFTMOTORLOG_SIZE  = 256;   // 256 entries = 2.5 seconds @ 10msec/100Hz
    protected double[]      liftMotorLogTime   = new double[LIFTMOTORLOG_SIZE];  // msec
    protected double[]      liftMotorLogAngle  = new double[LIFTMOTORLOG_SIZE];  // Angle [degrees]
    protected double[]      liftMotorLogPwr    = new double[LIFTMOTORLOG_SIZE];  // Power
    protected double[]      liftMotorLogAmps   = new double[LIFTMOTORLOG_SIZE];  // mAmp
    protected boolean       liftMotorLogEnable = false;
    protected int           liftMotorLogIndex  = 0;
    protected ElapsedTime   liftMotorTimer     = new ElapsedTime();
    
    //====== ENCODER RESET FLAG ======
    static private boolean transitionFromAutonomous = false;  // reset 1st time, plus anytime we do teleop-to-teleop

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

    //====== SERVOS FOR CONE GRABBER ====================================================================
    public Servo        leftTiltServo       = null;   // tilt GRABBER up/down (left arm)
    public Servo        rightTiltServo      = null;   // tilt GRABBER up/down (right arm)

    public double       currentTilt          =  0.00;  // This holds the most recent grabber tilt command
    public double       GRABBER_TILT_MAX     =  0.50;  // 0.5 (max) is up; -0.5 (min) is down
    public double       GRABBER_TILT_BACK_H  =  0.18;  // Backward scoring on the high pole
    public double       GRABBER_TILT_BACK_M  =  0.17;  // Backward scoring on the mid pole
    public double       GRABBER_TILT_INIT    = -0.04;  // Pointing straight up (overlaps front lift motor at some heights!)
    public double       GRABBER_TILT_SAFE    = -0.12;  // Maximum upward tilt that's safe to raise/lower collector past front lift motor
    public double       GRABBER_TILT_STORE   = -0.14;  // Stored angle for autonomous driving around
    public double       GRABBER_TILT_AUTO_F  = -0.21;  // 45deg tilt for front scoring in autonomous
    public double       GRABBER_TILT_FRONT_H_A = -0.18;  // Front scoring on the high pole auto
    public double       GRABBER_TILT_FRONT_H = -0.30;  // Front scoring on the high pole
    public double       GRABBER_TILT_FRONT_M_A = -0.34;  // Front scoring on the mid pole auto
    public double       GRABBER_TILT_FRONT_M = -0.28;  // Front scoring on the mid pole
    public double       GRABBER_TILT_FRONT_L = -0.28;  // Front scoring on the low pole
    public double       GRABBER_TILT_GRAB3   = -0.25;  // Further angled back for 5-stack drive-away (front) NO WALL CONFLICT!
    public double       GRABBER_TILT_GRAB2   = -0.30;  // Slightly angled up for collecting from 5-stack (front)
    public double       GRABBER_TILT_GRAB    = -0.34;  // Extended horizontal at ground level for grabbing (front)
    public double       GRABBER_TILT_MIN     = -0.50;  // As far down as we can tilt (manual control)

    public Servo        rotateServo         = null;   // rotate GRABBER left/right
    public double       GRABBER_ROTATE_UP   = 0.270;  // normal (upright) orientation
    public double       GRABBER_ROTATE_DOWN = 0.970;  // flipped (upside-down) orientation

    public CRServo      leftSpinServo       = null;   // continuous rotation/spin (left side)
    public CRServo      rightSpinServo      = null;   // continuous rotation/spin (right side)
    public double       GRABBER_PULL_POWER  = +0.50;
    public double       GRABBER_PUSH_POWER  = -0.50;

    //====== INFRARED PROXIMITY DETECTORS FOR CONE GRABBER ====================================================================
    public DigitalChannel topConeSensor;
    public boolean        topConeState = false;
    public DigitalChannel bottomConeSensor;
    public boolean        bottomConeState = false;
    //====== NAVIGATION DISTANCE SENSORS ================================================================
    private MaxSonarI2CXL sonarRangeL = null;   // Must include MaxSonarI2CXL.java in teamcode folder
    private MaxSonarI2CXL sonarRangeR = null;
    private MaxSonarI2CXL sonarRangeF = null;
    private MaxSonarI2CXL sonarRangeB = null;

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

    /* local OpMode members. */
    protected HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareSlimbot(){
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

        // Define and Initialize odometry pod encoders
//      leftOdometer   = hwMap.get(DcMotorEx.class,"LeftOdom");             // port0 (ideally a "REVERSE" motor port)
        rightOdometer  = hwMap.get(DcMotorEx.class,"RightOdom");  // port1 (ideally a "FORWARD" motor port)
//      strafeOdometer = hwMap.get(DcMotorEx.class,"StrafeOdom");           // port2 (ideally a "REVERSE" motor port)

        rightOdometer.setDirection(DcMotor.Direction.FORWARD);
//      leftOdometer.setDirection(DcMotor.Direction.REVERSE);
//      strafeOdometer.setDirection(DcMotor.Direction.REVERSE);

        // Zero all odometry encoders
        rightOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//      leftOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//      strafeOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Define and Initialize turret/lift motors
        turretMotor  = hwMap.get(DcMotorEx.class,"Turret");  // Expansion Hub port 3
        turretMotor.setDirection(DcMotor.Direction.FORWARD);
        turretMotor.setPower( 0.0 );
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretEncoder = hwMap.get(AnalogInput.class, "turretMA3");

        liftMotorF = hwMap.get(DcMotorEx.class,"LiftFront");   // Expansion Hub port 0
        liftMotorB = hwMap.get(DcMotorEx.class,"LiftBack");    // Expansion Hub port 1
        liftMotorF.setDirection(DcMotor.Direction.REVERSE);
        liftMotorB.setDirection(DcMotor.Direction.FORWARD);
        liftMotorsSetPower( 0.0 );
        liftMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // for odometry
        liftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // for odometry
        liftMotorF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftEncoder = hwMap.get(AnalogInput.class, "liftMA3");

        // Initialize servos
        leftTiltServo = hwMap.servo.get("leftTilt");      // servo port 0 (Control Hub)
        rightTiltServo = hwMap.servo.get("rightTilt");    // servo port 1 (Control Hub)
        grabberSetTilt( GRABBER_TILT_INIT );

        rotateServo = hwMap.servo.get("rotator");         // servo port 3 (Control Hub)
        rotateServo.setPosition( GRABBER_ROTATE_UP );

        leftSpinServo = hwMap.crservo.get("leftSpin");    // servo port 4 (Control Hub)
        leftSpinServo.setDirection( CRServo.Direction.REVERSE );
        leftSpinServo.setPower( 0.0 );

        rightSpinServo = hwMap.crservo.get("rightSpin");  // servo port 5 (Control Hub)
        rightSpinServo.setDirection( CRServo.Direction.FORWARD );
        rightSpinServo.setPower( 0.0 );

        // Initialize REV Expansion Hub IMU
        initIMU();

        //Instantiate Maxbotics ultrasonic range sensors (sensors wired to I2C ports)
//      sonarRangeL = hwMap.get( MaxSonarI2CXL.class, "leftUltrasonic" );   // I2C Bus 0
//      sonarRangeB = hwMap.get( MaxSonarI2CXL.class, "backUltrasonic" );   // I2C Bus 1
        sonarRangeF = hwMap.get( MaxSonarI2CXL.class, "frontUltrasonic" );  // I2C Bus 2
//      sonarRangeR = hwMap.get( MaxSonarI2CXL.class, "rightUltrasonic" );  // I2C Bus 3
        topConeSensor    = hwMap.get( DigitalChannel.class, "ProxTop" );
        bottomConeSensor = hwMap.get( DigitalChannel.class, "ProxBottom" );

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
        // Get a fresh set of values for this cycle
        //   getCurrentPosition() / getTargetPosition() / getTargetPositionTolerance()
        //   getPower() / getVelocity() / getCurrent()
        //===== CONTROL HUB VALUES =====
        frontLeftMotorPos  = frontLeftMotor.getCurrentPosition();
        frontLeftMotorVel  = frontLeftMotor.getVelocity();
        frontRightMotorPos = frontRightMotor.getCurrentPosition();
        frontRightMotorVel = frontRightMotor.getVelocity();
        rearRightMotorPos  = rearRightMotor.getCurrentPosition();
        rearRightMotorVel  = rearRightMotor.getVelocity();
        rearLeftMotorPos   = rearLeftMotor.getCurrentPosition();
        rearLeftMotorVel   = rearLeftMotor.getVelocity();
        turretAngle        = computeAbsoluteAngle( turretEncoder.getVoltage(), turretAngleOffset );
        liftAngle          = computeAbsoluteAngle( liftEncoder.getVoltage(),   liftAngleOffset );
        //===== EXPANSION HUB VALUES =====
        turretMotorPos     = turretMotor.getCurrentPosition();
        turretMotorVel     = turretMotor.getVelocity();
        turretMotorPwr     = turretMotor.getPower();
        double liftMotorPwrPrior = liftMotorPwr;
        liftMotorPwr       = liftMotorF.getPower();
        liftMotorRamp      = isPwrRampingDown( liftMotorPwrPrior, liftMotorPwr );
        // Parse right odometry encoder
        rightOdometerPrev  = rightOdometerCount;
        rightOdometerCount = rightOdometer.getCurrentPosition(); // Must be POSITIVE when bot moves FORWARD
        // Parse left odometry encoder
        leftOdometerPrev   = leftOdometerCount;
        leftOdometerCount  = -liftMotorB.getCurrentPosition();   // Must be POSITIVE when bot moves FORWARD
        // Parse rear odometry encoder
        strafeOdometerPrev  = strafeOdometerCount;
        strafeOdometerCount = -liftMotorF.getCurrentPosition();  // Must be POSITIVE when bot moves RIGHT
        // Get the cone sensor states
        topConeState    = topConeSensor.getState();
        bottomConeState = bottomConeSensor.getState();

        // NOTE: motor mA data is NOT part of the bulk-read, so increase cycle time!
//      frontLeftMotorAmps  = frontLeftMotor.getCurrent(MILLIAMPS);
//      frontRightMotorAmps = frontRightMotor.getCurrent(MILLIAMPS);
//      rearRightMotorAmps  = rearRightMotor.getCurrent(MILLIAMPS);
//      rearLeftMotorAmps   = rearLeftMotor.getCurrent(MILLIAMPS);
//      turretMotorAmps     = turretMotor.getCurrent(MILLIAMPS);
//      liftMotorAmps       = liftMotorF.getCurrent(MILLIAMPS) + liftMotorB.getCurrent(MILLIAMPS);

        // Do we need to capture lift motor instrumentation data?
        if( liftMotorLogEnable ) {
//          liftMotorAmps = liftMotorF.getCurrent(MILLIAMPS) + liftMotorB.getCurrent(MILLIAMPS);
            liftMotorLogTime[liftMotorLogIndex]  = liftMotorTimer.milliseconds();
            liftMotorLogAngle[liftMotorLogIndex] = liftAngle;
            liftMotorLogPwr[liftMotorLogIndex]   = liftMotorPwr;
            liftMotorLogAmps[liftMotorLogIndex]  = liftMotorAmps;
            // If the log is now full, disable further logging
            if( ++liftMotorLogIndex >= LIFTMOTORLOG_SIZE )
                liftMotorLogEnable = false;
        } // liftMotorLogEnable

        // Do we need to capture turret motor instrumentation data?
        if( turretMotorLogEnable ) {
//          turretMotorAmps = turretMotor.getCurrent(MILLIAMPS);
            turretMotorLogTime[turretMotorLogIndex]  = turretMotorTimer.milliseconds();
            turretMotorLogAngle[turretMotorLogIndex] = turretAngle;
            turretMotorLogPwr[turretMotorLogIndex]   = turretMotorPwr;
            turretMotorLogAmps[turretMotorLogIndex]  = turretMotorAmps;
            // If the log is now full, disable further logging
            if( ++turretMotorLogIndex >= TURRETMOTORLOG_SIZE )
                turretMotorLogEnable = false;
        } // turretMotorLogEnable

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
    public void turretMotorSetPower(double power) {
        // Are we stopping? (allow that no matter what the current turretAngle or power setting)
        if( Math.abs(power) <= 0.0001 ) {
                turretMotorRunning = false;
                turretMotorPwrSet = 0.0;
                turretMotor.setPower(0.0);
        }
        else {
        // Limit motor acceleration by clamping how big a change we can do in one cycle
        power = restrictDeltaPower( power, turretMotorPwrSet, 0.333 );
        // Positive turret power
          if(power > 0.0) {
            // Limit automatic turret movement to within the min/max safe range
            if(turretAngle < TURRET_ANGLE_MAX) {
                // We still have room to move, so set power.
                turretMotorRunning = true;
                turretMotorPwrSet = power;
                turretMotor.setPower(power);
            } else {
                // If we are at max angle, set the power to 0.
                turretMotorRunning = false;
                turretMotorPwrSet = 0.0;
                turretMotor.setPower(0.0);
            }
        // Negative turret power
        } else {
            // Limit automatic turret movement to within the min/max safe range
            if(turretAngle > TURRET_ANGLE_MIN) {
                // We still have room to move, so set power.
                turretMotorRunning = true;
                turretMotorPwrSet = power;
                turretMotor.setPower(power);
            } else {
                // If we are at max angle, set the power to 0.
                turretMotorRunning = false;
                turretMotorPwrSet = 0.0;
                turretMotor.setPower(0.0);
            }
        }
        }
    } // turretMotorSetPower

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
    public void liftMotorsSetPower( double motorPower )
    {
        // Are we stopping? (allow that no matter what the current liftAngle or power setting)
        if( Math.abs(motorPower) <= 0.0001 ) {
           liftMotorPwrSet = 0.0;
        }
        else {
           // Limit motor acceleration by clamping how big a change we can do in one cycle
           liftMotorPwrSet = restrictDeltaPower( motorPower, liftMotorPwrSet, 0.333 );
        }
        liftMotorF.setPower( liftMotorPwrSet );
        liftMotorB.setPower( liftMotorPwrSet );
    } // liftMotorsSetPower

    /*--------------------------------------------------------------------------------------------*/
    public void grabberSetTilt( double tiltAmount )
    {
        // Range-check the requested value
        if( tiltAmount >  0.50 ) tiltAmount =  0.50;
        if( tiltAmount < -0.50 ) tiltAmount = -0.50;
        // Rotate the two servos in opposite direction
        leftTiltServo.setPosition(  0.5 + tiltAmount );
        rightTiltServo.setPosition( 0.5 - tiltAmount );
        // Store the current setting
        currentTilt = tiltAmount;
    } // grabberSetTilt

    /*--------------------------------------------------------------------------------------------*/
    public void grabberSpinCollect()
    {
       leftSpinServo.setPower(  GRABBER_PULL_POWER );
       rightSpinServo.setPower( GRABBER_PULL_POWER );
    } // grabberSpinCollect

    public void grabberSpinEject()
    {
       leftSpinServo.setPower(  GRABBER_PUSH_POWER );
       rightSpinServo.setPower( GRABBER_PUSH_POWER );
    } // grabberSpinEject

    public void grabberSpinStop()
    {
       leftSpinServo.setPower(  0.0 );
       rightSpinServo.setPower( 0.0 );
    } // grabberSpinStop

    /**
     * @param p1 Motor power required to initiate rotation at LOW (v1) battery voltage
     * @param v1 LOW battery voltage data point (mV)
     * @param p2 Motor power required to initiate rotation at HIGH (v2) battery voltage
     * @param v2 HIGH battery voltage data point (mV)
     * @return Linear interpolated value for current battery state
     * Example: LOW 12.30V requires 0.0860; HIGH 13.54V requires 0.0650 for rotation 
     */
    public double getInterpolatedMinPower(double p1, double v1, double p2, double v2) {
        double slope = (p1 - p2) / (v1 - v2);
        double vNow = readBatteryExpansionHub();
        double pMin = slope * (vNow - v2) + p2;
        //TODO: bound pMin between p1 and p2 in case of bad battery reading
        return pMin;
    }

    /*--------------------------------------------------------------------------------------------*/
    /* turretPIDPosInit()                                                                         */
    /* - newAngle = desired turret angle                                                          */
    public void turretPIDPosInit( double newAngle )
    {
        // Current distance from target (degrees)
        double degreesToGo = newAngle - turretAngle;
        // pStatic = 0.0860 @ 12.30V 1  (117 rpm)
        // pStatic = 0.0650 @ 13.54V 2  (117 rpm)

        // pStatic = 0.0500 @ 12.8V 1  (43 rpm)
        // pStatic = 0.0400 @ 13.9V 2  (43 rpm)

        double pStatic = getInterpolatedMinPower(0.0500, 12800, 0.0400, 13900);

        turretPidController = new PIDControllerTurret( turretMotorPID_p, 
                                                       turretMotorPID_i,
                                                       turretMotorPID_d,
                                                       pStatic );

        // Are we ALREADY at the specified angle?
        if( Math.abs(degreesToGo) <= 1.0 )
            return;

        turretPidController.reset();

        // Ensure motor is stopped/stationary (aborts any prior unfinished automatic movement)
        turretMotorSetPower( 0.0 );

        // Establish a new target angle & reset counters
        turretMotorPIDAuto = true;
        turretAngleTarget = newAngle;
        turretMotorCycles = 0;
        turretMotorWait   = 0;

        // If logging instrumentation, begin a new dataset now:
        if( turretMotorLogging ) {
            turretMotorLogVbat   = readBatteryExpansionHub();
            turretMotorLogIndex  = 0;
            turretMotorLogEnable = true;
            turretMotorTimer.reset();
        }

    } // turretPIDPosInit

    /*--------------------------------------------------------------------------------------------*/
    /* turretPIDPosRun()                                                                          */
    public void turretPIDPosRun( boolean teleopMode )
    {
        // Has an automatic movement been initiated?
        if(turretMotorPIDAuto) {
            // Keep track of how long we've been doing this
            turretMotorCycles++;
            // Current distance from target (angle degrees)
            double degreesToGo = turretAngleTarget - turretAngle;
            double degreesToGoAbs = Math.abs(degreesToGo);
            int waitCycles = (teleopMode)? 2 : 2;
            double turretMotorPower = turretPidController.update( turretAngleTarget, turretAngle );
            turretMotorSetPower( turretMotorPower );
            // Have we achieved the target?
            // (temporarily limit to 16 cycles when verifying any major math changes!)
//          if( turretMotorCycles >= 16 ) {
            if( degreesToGoAbs <= 1.8 ) {   // there's +/-2deg of gear-mesh slop in 2:1 bevel gear
                turretMotorSetPower( 0.0 );
                if( ++turretMotorWait >= waitCycles ) {
                    turretMotorPIDAuto = false;
                    writeTurretLog();
                }
            }
            // No, still not within tolerance of desired target
            else {
                // Reset the wait count back to zero
                turretMotorWait = 0;
            }
        } // turretMotorPIDAuto

        // Has a fixed power movement been initiated?
        else if( turretMotorRunning ) {
            if((turretMotorPwrSet > 0) && (turretAngle >= TURRET_ANGLE_MAX)) {
                // If we are at max angle, set the power to 0.
                turretMotorSetPower(0.0);
            } else if((turretMotorPwrSet < 0) && (turretAngle <= TURRET_ANGLE_MIN)) {
                // If we are at max angle, set the power to 0.
                turretMotorSetPower(0.0);
            }
        } // turretMotorRunning

    } // turretPIDPosRun

    /*--------------------------------------------------------------------------------------------*/
    /* liftPIDPosInit()                                                                           */
    /* - newAngle = desired lift angle                                                            */
    public void liftPIDPosInit( double newAngle )
    {
        // Current distance from target (degrees)
        double degreesToGo = newAngle - liftAngle;
        double pStaticLift  = 0.320;
        double pStaticLower = 0.110;
        // Voltage doesn't seem as important on the arm minimum. Probably don't have to do
        // interpolated voltage. For example 0.13 power was not able to move arm at low voltage
        // and also could not at fresh battery voltage. 0.131 was able to at low voltage.
        // pStaticLower 0.130 @ 12.54V
        // pStaticLift  0.320 @ 12.81V
//        double pSin = getInterpolatedMinPower();

        liftPidController = new PIDControllerWormArm( liftMotorPIDr_p, liftMotorPIDr_i, liftMotorPIDr_d, 
                                                      liftMotorPIDr_f, pStaticLift,
                                                      liftMotorPIDl_p, liftMotorPIDl_i, liftMotorPIDl_d,
                                                      liftMotorPIDl_f, pStaticLower );

        // Are we ALREADY at the specified angle?
        if( Math.abs(degreesToGo) <= 1.0 )
            return;

        liftPidController.reset();

        // Ensure motor is stopped/stationary (aborts any prior unfinished automatic movement)
        liftMotorsSetPower( 0.0 );

        // Establish a new target angle & reset counters
        liftMotorPIDAuto = true;
        liftAngleTarget = newAngle;
        liftMotorCycles = 0;
        liftMotorWait   = 0;

        // If logging instrumentation, begin a new dataset now:
        if( liftMotorLogging ) {
            turretMotorLogVbat = readBatteryExpansionHub();
            liftMotorLogIndex  = 0;
            liftMotorLogEnable = true;
            liftMotorTimer.reset();
        }

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
            double degreesToGo = liftAngleTarget - liftAngle;
            double degreesToGoAbs = Math.abs(degreesToGo);
            int waitCycles = (teleopMode) ? 2 : 2;
            double power = liftPidController.update(liftAngleTarget, liftAngle);
            liftMotorsSetPower(power);
            // Have we achieved the target?
            // (temporarily limit to 16 cycles when verifying any major math changes!)
//          if( liftMotorCycles >= 16 ) {
            if( degreesToGoAbs <= 1.0 ) {
                liftMotorsSetPower( 0.0 );
                if( ++liftMotorWait >= waitCycles ) {
                    liftMotorPIDAuto = false;
                    writeLiftLog();
                }
            }
            // No, still not within tolerance of desired target
            else {
                // Reset the wait count back to zero
                liftMotorWait = 0;
            }
        } // liftMotorPIDAuto
    } // liftPIDPosRun

    public void performCycle(double scoreAngle)
    {
        // Now reverse the lift to raise off the cone stack
        liftPIDPosInit( LIFT_ANGLE_HIGH );
        turretPIDPosInit( scoreAngle );
        // Perform setup to center turret and raise lift to scoring position
        grabberSetTilt( GRABBER_TILT_FRONT_H );
    }

    public void resetCycle(double collectAngle)
    {
        liftPIDPosInit(LIFT_ANGLE_COLLECT);
        turretPIDPosInit( collectAngle );
        grabberSetTilt(GRABBER_TILT_GRAB);
    }

    /*--------------------------------------------------------------------------------------------*/
    /* liftPosInit()                                                                              */
    /* - newAngle = desired lift angle                                                            */
/*  public void liftPosInit( double newAngle )
    {
        // Current distance from target (degrees)
        double degreesToGo = newAngle - liftAngle;

        // Are we ALREADY at the specified angle?
        if( Math.abs(degreesToGo) < 2.0 )
            return;

        // Ensure motor is stopped/stationary (aborts any prior unfinished automatic movement)
        liftMotorsSetPower( 0.0 );

        // Establish a new target angle & reset counters
        liftMotorAuto   = true;
        liftAngleTarget = newAngle;
        liftMotorCycles = 0;
        liftMotorWait   = 0;

        // If logging instrumentation, begin a new dataset now:
        if( liftMotorLogging ) {
            liftMotorLogIndex  = 0;
            liftMotorLogEnable = true;
            liftMotorTimer.reset();
        }

    } // liftPosInit
*/
    /*--------------------------------------------------------------------------------------------*/
    /* liftPosRun()                                                                               */
/*  public void liftPosRun()
    {
        // Has an automatic movement been initiated?
        if( liftMotorAuto ) {
            // Keep track of how long we've been doing this
            liftMotorCycles++;
            // Current distance from target (angle degrees)
            double degreesToGo = liftAngleTarget - liftAngle;
            // Have we achieved the target?
            // (temporarily limit to 16 cycles when verifying any major math changes!)
//          if( liftMotorCycles >= 16 ) {
            if( Math.abs(degreesToGo) <= 0.8 ) {
                liftMotorsSetPower( 0.0 );
                if( ++liftMotorWait >= 2 ) {
                    liftMotorAuto = false;
                    writeLiftLog();
                }
            }
            // No, still not within tolerance of desired target
            else {
                // Are we moving on the front side? (LOWERING/LIFTING switch meaning on back side)
                boolean frontside = (liftAngle > 0.0)? true : false;
                // Are we LOWERING (little power required) or LIFTING (much power required)?
                boolean lowering = (frontside && (degreesToGo > 0.0)) ||
                                  (!frontside && (degreesToGo < 0.0));
                // Compute "base" motor power setting based on distance-from-target
                //   15 deg = 0.60 motor power (0.30 lowering | 0.90 lifting)
                //   10 deg = 0.50 motor power (0.25 lowering | 0.75 lifting)
                //    5 deg = 0.40 motor power (0.20 lowering | 0.60 lifting)
                //    1 deg = 0.32 motor power (0.16 lowering | 0.48 lifting)
                double minPower = (degreesToGo > 0.0)? -0.30 : +0.30;
                double liftMotorPower = minPower + (degreesToGo * -0.02); // our PID is just "P"
                // adjust base power according to lowering/lifting (lowering cuts it; raising boosts it)
                // when down near collect point we need more power, so don't reduce it as much
                double loweringFactor = (liftAngle > (LIFT_ANGLE_COLLECT-10.0))? 0.77 : 0.50;
                liftMotorPower *= (lowering)? loweringFactor : 1.50;
                // Ensure we don't request more than 100% motor power, even if long distance from target
                if( liftMotorPower >  LIFT_MOTOR_MAX ) liftMotorPower =  LIFT_MOTOR_MAX;
                if( liftMotorPower < -LIFT_MOTOR_MAX ) liftMotorPower = -LIFT_MOTOR_MAX;
                liftMotorsSetPower( liftMotorPower );
                // Reset the wait count back to zero
                liftMotorWait = 0;
            }
        } // liftMotorAuto
    } // liftPosRun
*/
    /*--------------------------------------------------------------------------------------------*/
    public void writeLiftLog() {
        // Are we even logging these events?
        if( !liftMotorLogging) return;
        // Movement must be complete (disable further logging to memory)
        liftMotorLogEnable = false;
        // Create a subdirectory based on DATE
        String dateString = new SimpleDateFormat("yyyy-MM-dd", Locale.getDefault()).format(new Date());
        String directoryPath = Environment.getExternalStorageDirectory().getPath() + "//FIRST//LiftMotor//" + dateString;
        // Ensure that directory path exists
        File directory = new File(directoryPath);
        directory.mkdirs();
        // Create a filename based on TIME
        String timeString = new SimpleDateFormat("hh-mm-ss", Locale.getDefault()).format(new Date());
        String filePath = directoryPath + "/" + "lift_" + timeString + ".txt";
        // Open the file
        FileWriter liftLog;
        try {
            liftLog = new FileWriter(filePath, false);
            liftLog.write("LiftMotor\r\n");
            liftLog.write("PIDr-p," + liftMotorPIDr_p + "\r\n");
            liftLog.write("PIDr-i," + liftMotorPIDr_i + "\r\n");
            liftLog.write("PIDr-d," + liftMotorPIDr_d + "\r\n");
            liftLog.write("PIDr-f," + liftMotorPIDr_f + "\r\n");
            liftLog.write("PIDl-p," + liftMotorPIDl_p + "\r\n");
            liftLog.write("PIDl-i," + liftMotorPIDl_i + "\r\n");
            liftLog.write("PIDl-d," + liftMotorPIDl_d + "\r\n");
            liftLog.write("PIDl-f," + liftMotorPIDl_f + "\r\n");
            liftLog.write("Battery Voltage," + turretMotorLogVbat + "\r\n");
            liftLog.write("Target Angle," + liftAngleTarget + "\r\n");
            // Log Column Headings
            liftLog.write("msec,pwr,mAmp,angle\r\n");
            // Log all the data recorded
            for( int i=0; i<liftMotorLogIndex; i++ ) {
                String msecString = String.format("%.3f, ", liftMotorLogTime[i] );
                String pwrString  = String.format("%.3f, ", liftMotorLogPwr[i]  );
                String ampString  = String.format("%.0f, ", liftMotorLogAmps[i] );
                String degString  = String.format("%.2f\r\n", liftMotorLogAngle[i]  );
                liftLog.write( msecString + pwrString + ampString + degString );
            }
            liftLog.flush();
            liftLog.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    } // writeLiftLog()

    /*--------------------------------------------------------------------------------------------*/
    /* turretPosInit()                                                                            */
    /* - newAngle = desired turret angle                                                          */
/*  public void turretPosInit( double newAngle )
    {
        // Current distance from target (degrees)
        double degreesToGo = newAngle - turretAngle;

        // Are we ALREADY at the specified angle?
        if( Math.abs(degreesToGo) < 1.0 )
            return;

        // Ensure motor is stopped/stationary (aborts any prior unfinished automatic movement)
        turretMotorSetPower( 0.0 );

        // Establish a new target angle & reset counters
        turretMotorAuto   = true;
        turretAngleTarget = newAngle;
        turretMotorCycles = 0;
        turretMotorWait   = 0;

        // If logging instrumentation, begin a new dataset now:
        if( turretMotorLogging ) {
            turretMotorLogVbat   = readBatteryExpansionHub();
            turretMotorLogIndex  = 0;
            turretMotorLogEnable = true;
            turretMotorTimer.reset();
        }

    } // turretPosInit
*/
    /*--------------------------------------------------------------------------------------------*/
    /* turretPosRun()                                                                             */
/*  public void turretPosRun( boolean teleopMode )
    {
        // Has an automatic movement been initiated?
        if( turretMotorAuto ) {
            // Keep track of how long we've been doing this
            turretMotorCycles++;
            // Current distance from target (angle degrees)
            double degreesToGo = turretAngleTarget - turretAngle;
            double degreesToGoAbs = Math.abs(degreesToGo);
            int waitCycles = (teleopMode)? 5 : 2;
            // Have we achieved the target?
            // (temporarily limit to 16 cycles when verifying any major math changes!)
//          if( turretMotorCycles >= 16 ) {
            if( degreesToGoAbs < 1.0 ) {
                turretMotorSetPower( 0.0 );
                if( ++turretMotorWait >= waitCycles ) {
                    turretMotorAuto = false;
                    writeTurretLog();
                }
            }
            // No, still not within tolerance of desired target
            else {
                // Reset the wait count back to zero
                turretMotorWait = 0;
                double turretMotorPower;
                if( teleopMode ) {  // Teleop (be FAST)
                    turretMotorPower = (degreesToGoAbs < 5.0)? 0.0 : (0.016 * degreesToGo);
                    turretMotorPower += (degreesToGo > 0)? 0.06 : -0.06;  // min power
                }
                else {  // Autonomous (be ACCURATE)
                    turretMotorPower = 0.011 * degreesToGo;
                    turretMotorPower += (degreesToGo > 0)? 0.06 : -0.06;  // min power
                }
                if( turretMotorPower < -0.75 ) turretMotorPower = -0.75;
                if( turretMotorPower > +0.75 ) turretMotorPower = +0.75;
                turretMotorSetPower( turretMotorPower );
            }
        } // turretMotorAuto

        // Has a fixed power movement been initiated?
        else if( turretMotorRunning ) {
            if((turretMotorPwrSet > 0) && (turretAngle >= TURRET_ANGLE_MAX)) {
                // If we are at max angle, set the power to 0.
                turretMotorSetPower(0.0);
            } else if((turretMotorPwrSet < 0) && (turretAngle <= TURRET_ANGLE_MIN)) {
                // If we are at max angle, set the power to 0.
                turretMotorSetPower(0.0);
            }
        } // turretMotorRunning

    } // turretPosRun
*/
    /*--------------------------------------------------------------------------------------------*/
    public void writeTurretLog() {
        // Are we even logging these events?
        if( !turretMotorLogging) return;
        // Movement must be complete (disable further logging to memory)
        turretMotorLogEnable = false;
        // Create a subdirectory based on DATE
        String dateString = new SimpleDateFormat("yyyy-MM-dd", Locale.getDefault()).format(new Date());
        String directoryPath = Environment.getExternalStorageDirectory().getPath() + "//FIRST//TurretMotor//" + dateString;
        // Ensure that directory path exists
        File directory = new File(directoryPath);
        directory.mkdirs();
        // Create a filename based on TIME
        String timeString = new SimpleDateFormat("hh-mm-ss", Locale.getDefault()).format(new Date());
        String filePath = directoryPath + "/" + "turret_" + timeString + ".txt";
        // Open the file
        FileWriter turretLog;
        try {
            turretLog = new FileWriter(filePath, false);
            turretLog.write("TurretMotor\r\n");
            turretLog.write("PID-p," + turretMotorPID_p + "\r\n");
            turretLog.write("PID-i," + turretMotorPID_i + "\r\n");
            turretLog.write("PID-d," + turretMotorPID_d + "\r\n");
            turretLog.write("Battery Voltage," + turretMotorLogVbat + "\r\n");
            turretLog.write("Target Angle," + turretAngleTarget + "\r\n");
            // Log Column Headings
            turretLog.write("msec,pwr,mAmp,angle\r\n");
            // Log all the data recorded
            for( int i=0; i<turretMotorLogIndex; i++ ) {
                String msecString = String.format("%.3f, ", turretMotorLogTime[i] );
                String pwrString  = String.format("%.3f, ", turretMotorLogPwr[i]  );
                String ampString  = String.format("%.0f, ", turretMotorLogAmps[i] );
                String degString  = String.format("%.2f\r\n", turretMotorLogAngle[i]  );
                turretLog.write( msecString + pwrString + ampString + degString );
            }
            turretLog.flush();
            turretLog.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    } // writeTurretLog()

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

    /*--------------------------------------------------------------------------------------------*/
    public int slowSonarRange( UltrasonicsInstances sensorInstance ) {
        // This is the SLOW version that sends an ultrasonic ping, waits 50 msec for the result,
        // and returns a value.  The returned valued is based on SINGLE reading (no averaging).
        // This version is intended for 1-time readings where the 50msec is okay (not control loops).
        int cm = 0;
        switch( sensorInstance ) {
//          case SONIC_RANGE_LEFT  : cm = sonarRangeL.getDistanceSync(); break;
//          case SONIC_RANGE_RIGHT : cm = sonarRangeR.getDistanceSync(); break;
            case SONIC_RANGE_FRONT : cm = sonarRangeF.getDistanceSync(); break;
//          case SONIC_RANGE_BACK  : cm = sonarRangeB.getDistanceSync(); break;
            default                : cm = 0;
        } // switch()
        return cm;
    } // slowSonarRange

    /*--------------------------------------------------------------------------------------------*/
    public int fastSonarRange( UltrasonicsInstances sensorInstance, UltrasonicsModes mode ) {
        // This is the FAST version that assumes there's a continuous sequence of pings being
        // triggered so it simply returns the "most recent" answer (no waiting!). This function is
        // intended for control loops that can't afford to incur a 50msec delay in the loop time.
        // The first call should pass SONIC_FIRST_PING and ignore the result; All subsequent calls
        // (assuming at least 50 msec has elapsed) should pass SONIC_MOST_RECENT and use the distance
        // returned.
        int cm = 0;
        switch( sensorInstance ) {
//          case SONIC_RANGE_LEFT  : cm = sonarRangeL.getDistanceAsync(); break;
//          case SONIC_RANGE_RIGHT : cm = sonarRangeR.getDistanceAsync(); break;
            case SONIC_RANGE_FRONT : cm = sonarRangeF.getDistanceAsync(); break;
//          case SONIC_RANGE_BACK  : cm = sonarRangeB.getDistanceAsync(); break;
            default                : cm = 0;
        } // switch()
        // Do we need to zero-out the value returned (likely from another time/place)?
        if( mode == HardwareSlimbot.UltrasonicsModes.SONIC_FIRST_PING ) {
            cm = 0;
        }
        // Return 
        return cm;
    } // fastSonarRange

    /*--------------------------------------------------------------------------------------------*/
    public int singleSonarRangeL() {
        // Query the current range sensor reading and wait for a response
        return sonarRangeL.getDistanceSync();
    } // singleSonarRangeL

    public int singleSonarRangeR() {
        // Query the current range sensor reading and wait for a response
        return sonarRangeR.getDistanceSync();
    } // singleSonarRangeR

    public int singleSonarRangeF() {
        // Query the current range sensor reading and wait for a response
        return sonarRangeF.getDistanceSync();
    } // singleSonarRangeF

    public int singleSonarRangeB() {
        // Query the current range sensor reading and wait for a response
        return sonarRangeB.getDistanceSync();
    } // singleSonarRangeB

    /*--------------------------------------------------------------------------------------------*/
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

} /* HardwareSlimbot */
