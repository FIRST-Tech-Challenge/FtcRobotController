package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Hardware class for goBilda robot (15"x15" chassis with 6" Andymark mecanum wheels)
 */
public class HardwareOneHub
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
    protected BNO055IMU imu = null;

    //====== MECANUM DRIVETRAIN MOTORS (RUN_USING_ENCODER) =====
    protected DcMotorEx frontLeftMotor     = null;
    public int          frontLeftMotorPos  = 0;        // current encoder count
    public double       frontLeftMotorVel  = 0.0;      // encoder counts per second
    private double      frontLeftLast      = 0.0;

    protected DcMotorEx frontRightMotor    = null;
    public int          frontRightMotorPos = 0;        // current encoder count
    public double       frontRightMotorVel = 0.0;      // encoder counts per second
    private double      frontRightLast     = 0.0;      // Track current motor power so we can limit abrupt changes

    protected DcMotorEx rearLeftMotor      = null;
    public int          rearLeftMotorPos   = 0;        // current encoder count
    public double       rearLeftMotorVel   = 0.0;      // encoder counts per second
    private double      rearLeftLast       = 0.0;

    protected DcMotorEx rearRightMotor     = null;
    public int          rearRightMotorPos  = 0;        // current encoder count
    public double       rearRightMotorVel  = 0.0;      // encoder counts per second
    private double      rearRightLast      = 0.0;

    //====== NAVIGATION DISTANCE SENSORS =====
//  private MaxSonarI2CXL sonar_left  = null;
//  private MaxSonarI2CXL sonar_right = null;

//  private DistanceSensor tofRangeL  = null;
//  private DistanceSensor tofRangeR  = null;

//  private int      sonarRangeLIndex   = 0;                          // 0...4
//  private double[] sonarRangeLSamples = {0,0,0,0,0};                // results of continuous sampling
//  private int      sonarRangeLSampCnt = sonarRangeLSamples.length;  // 5
//  private double   sonarRangeLMedian  = 0.0;
//  public  double   sonarRangeLStdev   = 0.0;

//  private int      sonarRangeRIndex   = 0;                          // 0...4
//  private double[] sonarRangeRSamples = {0,0,0,0,0};                // results of continuous sampling (most recent 5)
//  private int      sonarRangeRSampCnt = sonarRangeRSamples.length;  // 5
//  private double   sonarRangeRMedian  = 0.0;
//  public  double   sonarRangeRStdev   = 0.0;

//  private int      tofRangeLIndex     = 0;                          // 0...4
//  private double[] tofRangeLSamples   = {0,0,0,0,0};                // results of continuous sampling
//  private int      tofRangeLSampCnt   = tofRangeLSamples.length;  // 5
//  private double   tofRangeLMedian    = 0.0;
//  public  double   tofRangeLStdev     = 0.0;

//  private int      tofRangeRIndex     = 0;                          // 0...4
//  private double[] tofRangeRSamples   = {0,0,0,0,0};                // results of continuous sampling (most recent 5)
//  private int      tofRangeRSampCnt   = tofRangeRSamples.length;  // 5
//  private double   tofRangeRMedian    = 0.0;
//  public  double   tofRangeRStdev     = 0.0;

    /* local OpMode members. */
    protected HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareOneHub(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
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


        //Instantiate Maxbotics ultrasonic range sensors
//      sonar_left  = hwMap.get( MaxSonarI2CXL.class, "left_ultrasonic" );
//      sonar_right = hwMap.get( MaxSonarI2CXL.class, "right_ultrasonic" );

        //Instantiate REV 2-meter Time-Of-Flight Distance Sensors
//      tofRangeL   = hwMap.get( DistanceSensor.class, "ToF_distanceL" );
//      tofRangeR   = hwMap.get( DistanceSensor.class, "ToF_distanceR" );

        // Initialize REV Expansion Hub IMU
        initIMU();
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
        double heading = -(double)angles.firstAngle;
        return heading;  // degrees (+90 is CW; -90 is CCW)
    } // headingIMU

    /*--------------------------------------------------------------------------------------------*/
    public double headingIMUradians()
    {
        Orientation angles = imu.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS );
        double heading = -(double)angles.firstAngle;
        return heading;  // radians (+pi is CW; -pi is CCW)
    } // headingIMUradians

    /*--------------------------------------------------------------------------------------------*/
    public void readBulkData() {
        // For MANUAL mode, we must clear the BulkCache once per control cycle
 //     expansionHub.clearBulkCache();
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
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    } /* waitForTick() */

} /* HardwareOneHub */
