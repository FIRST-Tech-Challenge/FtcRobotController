package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.MILLIAMPS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
    protected BNO055IMU imu = null;

    //====== MECANUM DRIVETRAIN MOTORS (RUN_USING_ENCODER) =====
    protected DcMotorEx frontLeftMotor     = null;
    public int          frontLeftMotorTgt  = 0;       // RUN_TO_POSITION target encoder count
    public int          frontLeftMotorPos  = 0;       // current encoder count
    public double       frontLeftMotorVel  = 0.0;     // encoder counts per second

    protected DcMotorEx frontRightMotor    = null;
    public int          frontRightMotorTgt = 0;       // RUN_TO_POSITION target encoder count
    public int          frontRightMotorPos = 0;       // current encoder count
    public double       frontRightMotorVel = 0.0;     // encoder counts per second

    protected DcMotorEx rearLeftMotor      = null;
    public int          rearLeftMotorTgt   = 0;       // RUN_TO_POSITION target encoder count
    public int          rearLeftMotorPos   = 0;       // current encoder count
    public double       rearLeftMotorVel   = 0.0;     // encoder counts per second

    protected DcMotorEx rearRightMotor     = null;
    public int          rearRightMotorTgt  = 0;       // RUN_TO_POSITION target encoder count
    public int          rearRightMotorPos  = 0;       // current encoder count
    public double       rearRightMotorVel  = 0.0;     // encoder counts per second

    protected double COUNTS_PER_MOTOR_REV  = 28.0;    // goBilda Yellow Jacket Planetary Gear Motor Encoders
    protected double DRIVE_GEAR_REDUCTION  = 19.203;  // goBilda 19.2:1 (312rpm) gear ratio with 1:1 bevel gear
    protected double MECANUM_SLIPPAGE      = 1.01;    // one wheel revolution doesn't achieve 6" x 3.1415 of travel.
    protected double WHEEL_DIAMETER_INCHES = 6.0;     // For computing circumference
    protected double COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * MECANUM_SLIPPAGE) / (WHEEL_DIAMETER_INCHES * 3.1415);

    //====== DUCK CAROUSEL MOTOR (RUN_USING_ENCODER) =====
    protected DcMotorEx duckMotor     = null;
    public double       duckMotorVel  = 0.0;           // encoder counts per second

    //====== CAPPING ARM MOTOR (RUN_USING_ENCODER) =====
    protected DcMotorEx cappingMotor     = null;
    public int          cappingMotorTgt  = 0;          // RUN_TO_POSITION target encoder count
    public int          cappingMotorPos  = 0;          // current encoder count
    public double       cappingMotorAmps = 0.0;        // current power draw (Amps)

    public int          CAPPING_ARM_POS_START = 0;     // also used for STORE
    public int          CAPPING_ARM_POS_STORE = 349;
    public int          CAPPING_ARM_POS_CAP   = 1600;
    public int          CAPPING_ARM_POS_GRAB  = 2470;

    public int          cappingArmPos = CAPPING_ARM_POS_START;

    // CAPPING ARM WRIST SERVO
    public Servo   wristServo = null;
    public double  WRIST_SERVO_INIT   = 0.950;
    public double  WRIST_SERVO_GRAB   = 0.464;
    public double  WRIST_SERVO_STORE  = 0.269;
    public double  WRIST_SERVO_CAP    = 0.133;

    public Servo   clawServo = null;
    public double  CLAW_SERVO_INIT   = -0.10;
    public double  CLAW_SERVO_CLOSED = -0.10;
    public double  CLAW_SERVO_OPEN   =  0.30;

    //====== FREIGHT ARM MOTOR (RUN_USING_ENCODER) =====
    protected DcMotorEx freightMotor     = null;
    public int          freightMotorTgt  = 0;          // RUN_TO_POSITION target encoder count
    public int          freightMotorPos  = 0;          // current encoder count
    public double       freightMotorAmps = 0.0;        // current power draw (Amps)

    public int          FREIGHT_ARM_POS_COLLECT    = 0;     // Floor level (power-on position)
    public int          FREIGHT_ARM_POS_SPIN       = 50;    // Raised enough for box to spin clearly
    public int          FREIGHT_ARM_POS_SHARED     = 350;   // Front scoring into shared shipping hub
    public int          FREIGHT_ARM_POS_TRANSPORT1 = 400;   // Horizontal transport position
    public int          FREIGHT_ARM_POS_VERTICAL   = 1350;  // Vertical ("up" vs "down" reverse at this point)
    public int          FREIGHT_ARM_POS_HUB_TOP    = 2000;  // For dumping into hub top level
    public int          FREIGHT_ARM_POS_HUB_MIDDLE = 2275;  // For dumping into hub middle level
    public int          FREIGHT_ARM_POS_HUB_BOTTOM = 2400;  // For dumping into hub bottom level

    public Servo        boxServo                   = null;
    public double       BOX_SERVO_INIT             = 0.48;  // we init to the COLLECT position
    public double       BOX_SERVO_COLLECT          = 0.48;
    public double       BOX_SERVO_TRANSPORT        = 0.28;
    public double       BOX_SERVO_DUMP_TOP         = 0.55;
    public double       BOX_SERVO_DUMP_MIDDLE      = 0.55;
    public double       BOX_SERVO_DUMP_BOTTOM      = 0.55;
    public double       BOX_SERVO_DUMP_FRONT       = 0.80;  // ??

    public CRServo      sweepServo                 = null;  // CONTINUOUS, so no need for fixed positions

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
    public HardwareBothHubs(){
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

        //Initialize duck carousel motor
        duckMotor = hwMap.get(DcMotorEx.class,"DuckMotor");
        duckMotor.setDirection(DcMotor.Direction.REVERSE);  // goBilda fwd/rev opposite of Matrix motors!
        duckMotor.setPower( 0.0 );
        duckMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        duckMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duckMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Initialize capping arm motor
        cappingMotor = hwMap.get(DcMotorEx.class,"CappingMotor");
        cappingMotor.setDirection(DcMotor.Direction.REVERSE);  // goBilda fwd/rev opposite of Matrix motors!
        cappingMotor.setPower( 0.0 );
        cappingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cappingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cappingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wristServo = hwMap.servo.get("WristServo");    // servo port 0 (hub 2)
        wristServo.setPosition( WRIST_SERVO_INIT );

        clawServo = hwMap.servo.get("ClawServo");      // servo port 1 (hub 2)
        clawServo.setPosition( CLAW_SERVO_INIT );

        //Initialize freight arm motor
        freightMotor = hwMap.get(DcMotorEx.class,"FreightMotor");
        freightMotor.setDirection(DcMotor.Direction.REVERSE);  // goBilda fwd/rev opposite of Matrix motors!
        freightMotor.setPower( 0.0 );
        freightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        freightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        freightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boxServo = hwMap.servo.get("BoxServo");          // servo port 4 (hub 2)
        boxServo.setPosition( BOX_SERVO_INIT );

        sweepServo = hwMap.crservo.get("SweepServo");    // servo port 5 (hub 2)
        sweepServo.setDirection( CRServo.Direction.REVERSE );
        sweepServo.setPower( 0.0 );

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
        duckMotorVel       = duckMotor.getVelocity();
        cappingMotorPos    = cappingMotor.getCurrentPosition();
        cappingMotorAmps   = cappingMotor.getCurrent( MILLIAMPS );
        freightMotorPos    = freightMotor.getCurrentPosition();
        freightMotorAmps   = freightMotor.getCurrent( MILLIAMPS );
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
        // Are we ALREADY at the specific position?
        if( Math.abs(newPos-cappingMotorPos) < 20 )
           return;
        
        // Update the target position
        cappingMotorTgt = newPos;

        // Ensure motor is stopped/stationary and abort any prior RUN_TO_POSITION command
        cappingMotor.setPower( 0.0 );
        cappingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   
        // Configure new target encoder count
        cappingMotor.setTargetPosition(  cappingMotorTgt  );

        // Enable RUN_TO_POSITION mode
        cappingMotor.setMode(  DcMotor.RunMode.RUN_TO_POSITION );

        // Initiate motor movement to the new position
        cappingMotor.setPower( motorPower );

    } // cappingArmPosition
    
    /*--------------------------------------------------------------------------------------------*/
    /* freightArmPosition()                                                                       */
    /* - newPos     = desired new arm position                                                    */
    /* - motorPower = desired motor power level to get there                                      */
    public void freightArmPosition( int newPos, double motorPower )
    {
        // Are we ALREADY at the specific position?
        if( Math.abs(newPos-freightMotorPos) < 20 )
            return;

        // Update the target position
        freightMotorTgt = newPos;

        // Ensure motor is stopped/stationary and abort any prior RUN_TO_POSITION command
        freightMotor.setPower( 0.0 );
        freightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Configure new target encoder count
        freightMotor.setTargetPosition(  freightMotorTgt  );

        // Enable RUN_TO_POSITION mode
        freightMotor.setMode(  DcMotor.RunMode.RUN_TO_POSITION );

        // Initiate motor movement to the new position
        freightMotor.setPower( motorPower );

    } // freightArmPosition

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

} /* HardwareBothHubs */
