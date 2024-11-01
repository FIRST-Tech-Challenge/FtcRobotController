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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.HardwareDrivers.MaxSonarI2CXL;

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

    //====== Worm gear pan and tilt MOTORS (RUN_USING_ENCODER) =====
    protected DcMotorEx wormPanMotor     = null;
    public int          wormPanMotorTgt  = 0;       // RUN_TO_POSITION target encoder count
    public int          wormPanMotorPos  = 0;       // current encoder count
    public double       wormPanMotorVel  = 0.0;     // encoder counts per second
    public double       wormPanMotorAmps = 0.0;     // current power draw (Amps)

    protected DcMotorEx wormTiltMotor     = null;
    public int          wormTiltMotorTgt  = 0;       // RUN_TO_POSITION target encoder count
    public int          wormTiltMotorPos  = 0;       // current encoder count
    public double       wormTiltMotorVel  = 0.0;     // encoder counts per second
    public double       wormTiltMotorAmps = 0.0;     // current power draw (Amps)

    //====== Viper slide MOTOR (RUN_USING_ENCODER) =====
    protected DcMotorEx viperMotor       = null;
    public int          viperMotorTgt    = 0;       // RUN_TO_POSITION target encoder count
    public int          viperMotorPos    = 0;       // current encoder count
    public double       viperMotorVel    = 0.0;     // encoder counts per second
    public double       viperMotorAmps   = 0.0;     // current power draw (Amps)
    public double       viperMotorSetPwr = 0.0;     // requestedd power setting
    public double       viperMotorPwr    = 0.0;     // current power setting

    public boolean      viperMotorAutoMove = false;  // have we commanded an automatic lift movement?
    public boolean      viperMotorBusy     = false;
    public double       VIPER_RAISE_POWER  =  1.000; // Motor power used to RAISE viper slide
    public double       VIPER_HOLD_POWER   =  0.001; // Motor power used to HOLD viper slide at current height
    public double       VIPER_LOWER_POWER  = -0.500; // Motor power used to LOWER viper slide

    // Encoder counts for 435 RPM lift motors theoretical max 5.8 rev * 384.54 ticks/rev = 2230.3
    public int          VIPER_EXTEND_ZERO  = 0;      // 435 Encoder count when fully retracted (may need to be adjustable??)
    public int          VIPER_EXTEND_AUTO  = 482;    // 435 Encoder count when raised to just above the bin (safe to rotate - auto)
    public int          VIPER_EXTEND_BIN   = 519;    // 435 Encoder count when raised to just above the bin (safe to rotate - teleop)
    public int          VIPER_EXTEND_LOW   = 537;    // 435 Encoder count when raised to lowest possible scoring position
    public int          VIPER_EXTEND_MID   = 1038;   // 435 Encoder count when raised to medium scoring height
    public int          VIPER_EXTEND_HIGH  = 1482;   // 435 Encoder count when raised to upper scoring height
    public int          VIPER_EXTEND_FULL  = 2149;   // 435 Encoder count when fully extended (never exceed this count!)
    PIDControllerLift   liftPidController;           // PID parameters for the lift motors
    public double       liftMotorPID_p     = -0.100; //  Raise p = proportional
    public double       liftMotorPID_i     =  0.000; //  Raise i = integral
    public double       liftMotorPID_d     = -0.007; //  Raise d = derivative
    public boolean      liftMotorPIDAuto   = false;  // Automatic movement in progress (PID)
    public int          liftMotorCycles    = 0;      // Automatic movement cycle count
    public int          liftMotorWait      = 0;      // Automatic movement wait count (truly there! not just passing thru)
    public int          liftTarget         = 0;      // Automatic movement target ticks

    //====== COLLECTOR SERVOS =====
    public AnalogInput pushServoPos = null;
    public Servo  pushServo = null;
    public double PUSH_SERVO_INIT = 0.350;
    final public static double PUSH_SERVO_INIT_ANGLE = 230.0;
    public double PUSH_SERVO_SAFE = 0.350;  // Retract linkage servo back behind the pixel bin (safe to raise/lower)
    final public static double PUSH_SERVO_SAFE_ANGLE = 184.0;
    public double PUSH_SERVO_GRAB = 0.350;  // Partially extend to align fingers inside pixels
    final public static double PUSH_SERVO_GRAB_ANGLE = 168.0;
    public double PUSH_SERVO_DROP = 0.350;  // Fully extend finger assembly toward the Backdrop
    final public static double PUSH_SERVO_DROP_ANGLE = 56.1;
    final public static double PUSH_SERVO_PIXEL_CLEAR_ANGLE = 90.0; // Pulling back from backdrop but cleared pixel

    public AnalogInput wristServoPos = null;
    public Servo  wristServo = null;
    public double WRIST_SERVO_INIT = 0.500;           // higher is counter-clockwise
    final public static double WRIST_SERVO_INIT_ANGLE = 188.0; // no idea yet, will have to figure it out!
    public double WRIST_SERVO_GRAB = 0.500;
    final public static double WRIST_SERVO_GRAB_ANGLE = 183.5;
    public double WRIST_SERVO_DROP = 0.500;
    final public static double WRIST_SERVO_DROP_ANGLE = 128.9;

    //Ultrasonic sensors
    private MaxSonarI2CXL sonarRangeF = null;

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

        wormPanMotor.setDirection(DcMotor.Direction.FORWARD);
        wormTiltMotor.setDirection(DcMotor.Direction.FORWARD);

        wormPanMotor.setPower( 0.0 );
        wormTiltMotor.setPower( 0.0 );

        wormPanMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormTiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wormPanMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wormTiltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wormPanMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wormTiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Viper slide motor
        viperMotor = hwMap.get(DcMotorEx.class,"viperMotor");  // Expansion Hub port 2
        viperMotor.setDirection(DcMotor.Direction.REVERSE);   // positive motor power extends
        viperMotor.setPower( 0.0 );
        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pushServo = hwMap.servo.get("ElbowServo");             // servo port 0 (Expansion Hub)
        pushServoPos = hwMap.analogInput.get("ElbowServoPos"); // Analog port 1 (Expansion Hub)
        pushServo.setPosition(PUSH_SERVO_INIT);

        wristServo = hwMap.servo.get("WristServo");             // servo port 1 (Expansion Hub)
        wristServoPos = hwMap.analogInput.get("WristServoPos"); // Analog port 0 (Expansion Hub)
        wristServo.setPosition(WRIST_SERVO_INIT);

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
        frontLeftMotorPos  = frontLeftMotor.getCurrentPosition();
        frontLeftMotorVel  = frontLeftMotor.getVelocity();
        frontRightMotorPos = frontRightMotor.getCurrentPosition();
        frontRightMotorVel = frontRightMotor.getVelocity();
        rearRightMotorPos  = rearRightMotor.getCurrentPosition();
        rearRightMotorVel  = rearRightMotor.getVelocity();
        rearLeftMotorPos   = rearLeftMotor.getCurrentPosition();
        rearLeftMotorVel   = rearLeftMotor.getVelocity();
        viperMotorPos      = viperMotor.getCurrentPosition();
        viperMotorVel      = viperMotor.getVelocity();
        viperMotorPwr      = viperMotor.getPower();
        // NOTE: motor mA data is NOT part of the bulk-read, so increases cycle time!
//      frontLeftMotorAmps  = frontLeftMotor.getCurrent(MILLIAMPS);
//      frontRightMotorAmps = frontRightMotor.getCurrent(MILLIAMPS);
//      rearRightMotorAmps  = rearRightMotor.getCurrent(MILLIAMPS);
//      rearLeftMotorAmps   = rearLeftMotor.getCurrent(MILLIAMPS);
//      viperMotorAmps      = viperMotor.getCurrent(MILLIAMPS);

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
    } // viperSlideExtension

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
            liftMoveState = LiftMoveActivity.IDLE;
            liftStoreState = LiftStoreActivity.IDLE;
           viperMotorAutoMove = false;
           viperMotorBusy = false;
        }
    } // abortViperSlideExtension


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
        if( mode == Hardware2025Bot.UltrasonicsModes.SONIC_FIRST_PING ) {
            cm = 0;
        }
        // Return
        return cm;
    } // fastSonarRange

    /*--------------------------------------------------------------------------------------------*/
    public double getPushServoAngle() {
        return (pushServoPos.getVoltage() / 3.3) * 360.0;
    }

    public double getWristServoAngle() {
        return (wristServoPos.getVoltage() / 3.3) * 360.0;
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
            viperMotorSetPwr = 0.0;
        }
        else {
            // Limit motor acceleration by clamping how big a change we can do in one cycle
            viperMotorSetPwr = restrictDeltaPower( motorPower, viperMotorSetPwr, 0.333 );
        }
        viperMotor.setPower( viperMotorSetPwr );
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
        int ticksToGo = newPosition - viperMotorPos;
        double pStaticLift  = 0.007;
        // Voltage doesn't seem as important on the arm minimum. Probably don't have to do
        // interpolated voltage. For example 0.13 power was not able to move arm at low voltage
        // and also could not at fresh battery voltage. 0.131 was able to at low voltage.
        // pStaticLower 0.130 @ 12.54V
        // pStaticLift  0.320 @ 12.81V
//      double pSin = getInterpolatedMinPower();

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
//      if( liftMotorLogging ) {
//          turretMotorLogVbat = readBatteryExpansionHub();
//          liftMotorLogIndex  = 0;
//          liftMotorLogEnable = true;
//          liftMotorTimer.reset();
//      }

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
            int ticksToGo = liftTarget - viperMotorPos;
            int ticksToGoAbs = Math.abs(ticksToGo);
            int waitCycles = (teleopMode) ? 2 : 2;
            double power = liftPidController.update(liftTarget, viperMotorPos);
            liftMotorsSetPower(power);
            // Have we achieved the target?
            // (temporarily limit to 16 cycles when verifying any major math changes!)
//          if( liftMotorCycles >= 16 ) {
            if( ticksToGoAbs <= 10 ) {
                liftMotorsSetPower( 0.0 );
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

} /* Hardware2025Bot */
