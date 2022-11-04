package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.MILLIAMPS;
import static java.lang.Thread.sleep;

import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
//  protected double DRIVE_GEAR_REDUCTION  = 26.851;  // goBilda 26.9:1 (223rpm) gear ratio with 1:1 bevel gear
    protected double DRIVE_GEAR_REDUCTION  = 19.203;  // goBilda 19.2:1 (312rpm) gear ratio with 1:1 bevel gear
    protected double MECANUM_SLIPPAGE      = 1.01;    // one wheel revolution doesn't achieve 6" x 3.1415 of travel.
    protected double WHEEL_DIAMETER_INCHES = 3.77953; // (96mm) -- for computing circumference
    protected double COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * MECANUM_SLIPPAGE) / (WHEEL_DIAMETER_INCHES * 3.1415);

    //====== MOTORS FOR GAMEPLAY MECHANISMS (turret / lift) =====
    protected DcMotorEx turretMotor        = null;    // A pair of motors operated as one with a Y cable
    public int          turretMotorTgt     = 0;       // RUN_TO_POSITION target encoder count
    public int          turretMotorPos     = 0;       // current encoder count
    public double       turretMotorVel     = 0.0;     // encoder counts per second
    public double       turretMotorAmps    = 0.0;     // current power draw (Amps)

    protected AnalogInput turretEncoder    = null;    // US Digital absolute magnetic encoder (MA3)
    public double       turretAngle        = 0.0;     // 0V = 0 degrees; 3.3V = 359.99 degrees
    public double       turretAngleOffset  = 299.0;   // allows us to adjust the 0-360 deg range

    public int          TURRET_LIMIT_LEFT  = -999;    // encoder count at maximum rotation LEFT
    public int          TURRET_LIMIT_RIGHT = +999;    // encoder count at maximum rotation RIGHT

    protected DcMotorEx liftMotorF         = null;    // FRONT lift motor
    protected DcMotorEx liftMotorB         = null;    // BACK lift motor
    public boolean      liftMotorAuto      = false;   // Automatic movement in progress
    public int          liftMotorCycles    = 0;       // Automatic movement cycle count
    public int          liftMotorWait      = 0;       // Automatic movement wait count (truly there! not just passing thru)
    public double       liftMotorPwr       = 0.0;     // lift motors power setpoint (-1.0 to +1.0)
    public double       liftMotorAmps      = 0.0;     // lift motors current power draw (Amps)
    public boolean      liftMotorRamp      = false;   // motor power setting is ramping down
    
    protected AnalogInput liftEncoder      = null;    // US Digital absolute magnetic encoder (MA3)
    public double       liftAngle          = 0.0;     // 0V = 0 degrees; 3.3V = 359.99 degrees
    public double       liftAngleOffset    = 157.5;   // allows us to adjust the 0-360 deg range
    public double       liftAngleTarget    = 0.0;     // Automatic movement target angle (degrees)

    public double       LIFT_ANGLE_MAX     =  92.0;   // absolute encoder angle at maximum rotation FRONT
    public double       LIFT_ANGLE_MIN     = -20.0;   // absolute encoder angle at maximum rotation REAR
    // NOTE: the motor doesn't stop immediately, so a limit of 115 deg halts motion around 110 degrees
    public double       LIFT_ANGLE_COLLECT = 89.0;    // lift position for collecting cones
    public double       LIFT_ANGLE_GROUND  = 88.0;    // lift position for collecting cones
    public double       LIFT_ANGLE_LOW     = 61.0;    // lift position for LOW junction
    public double       LIFT_ANGLE_MED     = 42.0;    // lift position for MEDIUM junction
    public double       LIFT_ANGLE_HIGH    = 12.0;    // lift position for HIGH junction

    // Instrumentation:  writing to input/output is SLOW, so to avoid impacting loop time as we capture
    // motor performance we store data to memory until the movement is complete, then dump to a file.
    public boolean          liftMotorLogging   = false; // only enable during development!!
    public final static int LIFTMOTORLOG_SIZE  = 128;   // 128 entries = 2+ seconds @ 16msec/60Hz
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

    public double       GRABBER_TILT_MAX    =  0.50;  // 0.5 (max) is up; -0.5 (min) is down
    public double       GRABBER_TILT_STORE  =  0.30;
    public double       GRABBER_TILT_INIT   =  0.35;
    public double       GRABBER_TILT_GRAB   =  0.20;
    public double       GRABBER_TILT_MIN    = -0.50;

    public Servo        rotateServo         = null;   // rotate GRABBER left/right
    public double       GRABBER_ROTATE_UP   = 0.335;  // normal (upright) orientation
    public double       GRABBER_ROTATE_DOWN = 1.000;  // flipped (upside-down) orientation

    public CRServo      leftSpinServo       = null;   // continuous rotation/spin (left side)
    public CRServo      rightSpinServo      = null;   // continuous rotation/spin (right side)
    public double       GRABBER_PULL_POWER  = +0.50;
    public double       GRABBER_PUSH_POWER  = -0.50;

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
    //  leftOdometer    = hwMap.get(DcMotorEx.class,"LeftOdom");   // port0 (ideally a "REVERSE" motor port)
        rightOdometer   = hwMap.get(DcMotorEx.class,"RightOdom");  // port1 (ideally a "FORWARD" motor port)
//      strafeOdometer  = hwMap.get(DcMotorEx.class,"StrafeOdom");           // port2 (ideally a "REVERSE" motor port)

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
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretEncoder = hwMap.get(AnalogInput.class, "turretMA3");

        liftMotorF = hwMap.get(DcMotorEx.class,"LiftFront");   // Expansion Hub port 0
        liftMotorB = hwMap.get(DcMotorEx.class,"LiftBack");    // Expansion Hub port 1
        liftMotorF.setDirection(DcMotor.Direction.REVERSE);
        liftMotorB.setDirection(DcMotor.Direction.FORWARD);
        liftMotorsSetPower( 0.0 );
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
        sonarRangeL = hwMap.get( MaxSonarI2CXL.class, "leftUltrasonic" );   // I2C Bus 0
        sonarRangeB = hwMap.get( MaxSonarI2CXL.class, "backUltrasonic" );   // I2C Bus 1
        sonarRangeF = hwMap.get( MaxSonarI2CXL.class, "frontUltrasonic" );  // I2C Bus 2
//      sonarRangeR = hwMap.get( MaxSonarI2CXL.class, "rightUltrasonic" );  // I2C Bus 3

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
        turretAngle        = computeAbsoluteAngle( turretEncoder.getVoltage(), turretAngleOffset );
        liftAngle          = computeAbsoluteAngle( liftEncoder.getVoltage(),   liftAngleOffset );
        //===== EXPANSION HUB VALUES =====
        turretMotorPos     = turretMotor.getCurrentPosition();
        turretMotorVel     = turretMotor.getVelocity();
        turretMotorAmps    = turretMotor.getCurrent(MILLIAMPS);
        liftMotorAmps      = liftMotorF.getCurrent(MILLIAMPS) + liftMotorB.getCurrent(MILLIAMPS);
        double liftMotorPwrPrior = liftMotorPwr;
        liftMotorPwr       = liftMotorF.getPower();
        liftMotorRamp      = isPwrRampingDown( liftMotorPwrPrior, liftMotorPwr );
        // Parse right odometry encoder
        rightOdometerPrev  = rightOdometerCount;
        rightOdometerCount = rightOdometer.getCurrentPosition(); // Must be POSITIVE when bot moves FORWARD
        // Parse left odometry encoder
        leftOdometerPrev   = leftOdometerCount;
//      leftOdometerCount  = leftOdometer.getCurrentPosition();  // Must be POSITIVE when bot moves FORWARD
        // Parse rear odometry encoder
        strafeOdometerPrev  = strafeOdometerCount;
//      strafeOdometerCount = strafeOdometer.getCurrentPosition();

        // Do we need to capture lift motor instrumentation data?
        if( liftMotorLogEnable ) {
            liftMotorLogTime[liftMotorLogIndex]  = liftMotorTimer.milliseconds();
            liftMotorLogAngle[liftMotorLogIndex] = liftAngle;
            liftMotorLogPwr[liftMotorLogIndex]   = liftMotorPwr;
            liftMotorLogAmps[liftMotorLogIndex]  = liftMotorAmps;
            // If the log is now full, disable further logging
            if( ++liftMotorLogIndex >= LIFTMOTORLOG_SIZE )
                liftMotorLogEnable = false;
        } // liftMotorLogEnable

    } // readBulkData

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
    public void stopMotion() {
        // Stop all motion;
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
    }

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
        liftMotorF.setPower( motorPower );
        liftMotorB.setPower( motorPower );
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

    /*--------------------------------------------------------------------------------------------*/
    /* liftPosInit()                                                                              */
    /* - newAngle = desired lift angle                                                            */
    public void liftPosInit( double newAngle )
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

    /*--------------------------------------------------------------------------------------------*/
    /* liftPosRun()                                                                               */
    public void liftPosRun()
    {
        // Has an automatic movement been initiated?
        if( liftMotorAuto ) {
            // Keep track of how long we've been doing this
            liftMotorCycles++;
            // Current distance from target (angle degrees)
            double degreesToGo = liftAngleTarget - liftAngle;
            // Have we achieved the target?
//          if( liftMotorCycles >= 16 ) {
            if( Math.abs(degreesToGo) < 1.0 ) {
                liftMotorsSetPower( 0.0 );
                if( ++liftMotorWait >= 2 ) {
                    liftMotorAuto = false;
                    writeLiftLog();
                }
            }
            // No, still not within tolerance of desired target
            else {
                double liftMotorPower;
                // Reset the wait count back to zero
                liftMotorWait = 0;
                // Are we LOWERING, but quite a ways from target
                if( degreesToGo > 10.0 )
                    liftMotorPower = -0.50;
                // No abrupt changes when lowering, or we'll slip a gear tooth
                else if( degreesToGo > 5.0 )
                    liftMotorPower = -0.40;
                // Are we LOWERING but within 5deg of target?
                else if( degreesToGo > 0 )
                    liftMotorPower = -0.30;
                // Are we RAISING but within 5deg of target?
                else if( degreesToGo > -5.0 )
                    liftMotorPower = 0.40;
                // Otherwise we're RAISING, but quite a ways to go
                else
                    liftMotorPower = 0.80;
                liftMotorsSetPower( liftMotorPower );
            }
        } // liftMotorAuto
    } // liftPosRun

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
