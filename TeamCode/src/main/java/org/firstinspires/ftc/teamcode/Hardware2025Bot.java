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
//import com.qualcomm.robotcore.hardware.CRServo;
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
    public double imuTiltAngle    = 0.0;

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
    public double       WORM_TILT_VIPER_TO_BASKET_DEG= 45.0;   // angle at when the viper motor starts extending when going to basket
    public double       WORM_TILT_VIPER_FROM_BASKET_DEG  = 80; // angle at when the viper motor starts extending from the basket

    protected AnalogInput armTiltEncoder   = null;    // US Digital absolute magnetic encoder (MA3)
    public double           armTiltAngle   = 0.0;     // 0V = 0 degrees; 3.3V = 359.99 degrees
    public static final double armTiltAngleOffset = 129.0;     // allows us to adjust the 0-360 deg range
    public double       turretAngleTarget   = 0.0;     // Automatic movement target angle (degrees)

    // This value is set at init.
    public static double startingArmTiltAngle = 0.0;
    // Delta math from -0.1 deg -3891 encoder counts
    //                  94.4 deg 5 encoder counts
    //                  94.5 deg 3896 encoder counts range
    public final static double ENCODER_COUNTS_PER_DEG  = 3896.0 / 94.5;

    public final static double TILT_ANGLE_HW_MAX_DEG      = 90.00; // Arm at maximum rotation UP/BACK (horizontal = -200)
    public final static double TILT_ANGLE_BASKET_DEG      = 90.00; // Arm at rotation back to the basket for scoring
    public final static double TILT_ANGLE_AUTO_PRE_DEG    = 83.00; // Arm almost at  basket (start to slow; avoid wobble)
    public final static double TILT_ANGLE_ASCENT1_DEG     = 93.00; // Arm at rotation back to the low bar for ascent level 1 or 2
    public final static double TILT_ANGLE_ASCENT2_DEG     = 75.00; // Arm at rotation back to the low bar for ascent level 2
    public final static double TILT_ANGLE_PARK_DEG        = 33.80; // Arm at rotation back to the low bar for park in auto
    public final static double TILE_ANGLE_BASKET_SAFE_DEG = 90.00; // Arm safe to rotate intake from basket
    public final static double TILT_ANGLE_VERTICAL_DEG    = 54.50; // Straight up vertical (safe to start retracting viper)
    public final static double TILT_ANGLE_ZERO_DEG        =  6.00; // Arm for parking fully reset in auto
    public final static double TILT_ANGLE_DRIVE_DEG       =  6.00; // Arm for parking in auto or driving around
    public final static double TILT_ANGLE_SPECIMEN0_DEG   = 60.00; // (NEW) Angle for grabbing specimens off field wall
    public final static double TILT_ANGLE_SPECIMEN1_DEG   = 65.00; // AUTO: Angle for scoring specimens (above bar)
    public final static double TILT_ANGLE_SPECIMEN2_DEG   = 59.40; // AUTO: Angle for scoring specimens (clipped)
    public final static double TILT_ANGLE_CLIP_DEG        = 45.00; // AUTO: clip specimen on bar by just driving forward
    public final static double TILT_ANGLE_HW_MIN_DEG      =  0.00; // Arm at maximum rotation DOWN/FWD
    public final static double TILT_ANGLE_COLLECT_DEG     =  6.00; // Arm to collect samples at ground level
    public final static double TILT_ANGLE_SAMPLE3_DEG     =  6.00; // Arm to collect samples at ground level (3rd one against wall)
    public final static double TILT_ANGLE_START_DEG       = 13.00; // AUTO: starting position LOW
    public final static double TILT_ANGLE_WALL_DEG        = 13.90; // AUTO: starting position HIGH (motor tilted back & touches wall)
    public final static double TILT_ANGLE_WALL0_DEG       = 21.50; // AUTO: grab specimen off wall (on approach)
    public final static double TILT_ANGLE_WALL1_DEG       = 33.00; // AUTO: grab specimen off wall (lift off)

    //====== Viper slide MOTOR (RUN_USING_ENCODER) =====
    protected DcMotorEx viperMotor       = null;
    public int          viperMotorTgt    = 0;       // RUN_TO_POSITION target encoder count
    public int          viperMotorPos    = 0;       // current encoder count
    public double       viperMotorVel    = 0.0;     // encoder counts per second
    public double       viperMotorAmps   = 0.0;     // current power draw (Amps)
    public double       viperMotorAmpsPk = 0.0;     // peak power draw (Amps) 312rpm = 9.2A @ 12V
    public double       viperMotorSetPwr = 0.0;     // requested power setting
    public double       viperMotorPwr    = 0.0;     // current power setting

    public ElapsedTime  viperSlideTimer  = new ElapsedTime();
    public ElapsedTime  wormTiltTimer    = new ElapsedTime();

    public boolean      viperMotorAutoMove = false;    // have we commanded an automatic extension movement?
    public boolean      viperMotorBusy     = false;
    public boolean      wormTiltMotorAutoMove = false; // have we commanded an automatic tilt movement?
    public boolean      wormTiltMotorBusy     = false;
    public double       VIPER_RAISE_POWER  =  1.000; // Motor power used to EXTEND viper slide
    public double       VIPER_HOLD_POWER   =  0.001; // Motor power used to HOLD viper slide at current extension
    public double       VIPER_LOWER_POWER  = -0.500; // Motor power used to RETRACT viper slide

    // Encoder counts for 435 RPM lift motors theoretical max 5.8 rev * 384.54 ticks/rev = 2230 counts
    // Encoder counts for 312 RPM lift motors theoretical max 5.8 rev * 537.7  ticks/rev = 3118 counts
    // Encoder counts for 223 RPM lift motors theoretical max 5.8 rev * 751.8  ticks/rev = 4360 counts

    public final static int    VIPER_EXTEND_ZERO  = 0;      // fully retracted (may need to be adjustable??)
    public final static int    VIPER_EXTEND_AUTO_READY  = 1600;    // extend for collecting during auto
    public final static int    VIPER_EXTEND_AUTO_COLLECT = 1600;    // extend for collecting during auto
    public final static int    VIPER_EXTEND_SAMPLE3  = 1500;  // extend for collecting during auto (3rd sample along wall)
    public final static int    VIPER_EXTEND_HANG1 = 1250;   // extend to this to prepare for level 2 ascent
    public final static int    VIPER_EXTEND_PARK = 3410;   // extend to this to park in auto
    public final static int    VIPER_EXTEND_HANG2 =  602;   // retract to this extension during level 2 ascent
    public final static int    VIPER_EXTEND_GRAB  = 1600;   // extend for collection from submersible
    public final static int    VIPER_EXTEND_SECURE=  490;   // Intake is tucked into robot to be safe
    public final static int    VIPER_EXTEND_SAFE  = 1100;   // Intake is far enough out to safely rotate down and rotate up
    public final static int    VIPER_EXTEND_AUTO1 = 1980;   // NEW raised to where the specimen hook is above the high bar
    public final static int    VIPER_EXTEND_AUTO2 = 1200;   // NEW retract to clip the specimen to the bar
    public final static int    VIPER_EXTEND_CLIP  = 1580;   // AUTO: clip specimen on bar by just driving forward
    public final static int    VIPER_EXTEND_BASKET= 4200;   // raised to basket-scoring height
    public final static int    VIPER_EXTEND_FULL1 = 2800;   // extended 36" forward (max for 20"x42" limit at horizontal)
    public final static int    VIPER_EXTEND_FULL2 = 4214;   // hardware fully extended (never exceed this count!)
    public final static int    VIPER_EXTEND_WALL0 = 25;     // AUTO: grab specimen off wall (on approach)
    public final static int    VIPER_EXTEND_WALL1 = 230;    // AUTO: grab specimen off wall (lift off)

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

    public final static double ELBOW_SERVO_INIT = 0.500;
    public final static double ELBOW_SERVO_INIT_ANGLE = 180.0;
    public final static double ELBOW_SERVO_SAFE = 0.510;       // Safe orientation for driving
    public final static double ELBOW_SERVO_SAFE_ANGLE = 180.0;
    public final static double ELBOW_SERVO_GRAB = 0.510;       // For grabbing samples from the field floor
    public final static double ELBOW_SERVO_GRAB_ANGLE = 180.0;
    public final static double ELBOW_SERVO_GRAB3 = 0.580;      // For grabbing 3rd sample from field floor (against wall)
    public final static double ELBOW_SERVO_GRAB3_ANGLE = 180.0;
    public final static double ELBOW_SERVO_BASKET = 0.500;     // For scoring samples in the basket
    public final static double ELBOW_SERVO_BASKET_ANGLE = 180.0;
    public final static double ELBOW_SERVO_BAR1 = 0.520;       // NEW specimen bar (above)
    public final static double ELBOW_SERVO_BAR1_ANGLE = 174.0;
    public final static double ELBOW_SERVO_BAR2 = 0.520;       // NEW specimen bar (clipped)
    public final static double ELBOW_SERVO_BAR2_ANGLE = 174.0;
    public final static double ELBOW_SERVO_WALL0 = 0.500;       // Grab specimen off wall in autonomous
    public final static double ELBOW_SERVO_WALL0_ANGLE = 180.0; // Grab specimen off wall in autonomous
    public final static double ELBOW_SERVO_WALL1 = 0.510;       // Grab specimen off wall in autonomous
    public final static double ELBOW_SERVO_WALL1_ANGLE = 176.0; // Grab specimen off wall in autonomous
    public final static double ELBOW_SERVO_WALL2 = 0.500;       // Grab specimen off wall in autonomous
    public final static double ELBOW_SERVO_WALL2_ANGLE = 180.0; // Grab specimen off wall in autonomous
    public final static double ELBOW_SERVO_CLIP = 0.510;        // AUTO: clip specimen on bar by just driving forward

    public AnalogInput wristServoPos = null;
    public Servo  wristServo = null;

    public final static double WRIST_SERVO_INIT = 0.159;        // stored (pointing up)
    public final static double WRIST_SERVO_INIT_ANGLE = 288.0;
    public final static double WRIST_SERVO_SAFE = 0.340;        // safe orientation for driving
    public final static double WRIST_SERVO_SAFE_ANGLE = 234.0;
    public final static double WRIST_SERVO_GRAB = 0.730;        // grab floor sample (pointing down)
    public final static double WRIST_SERVO_GRAB_ANGLE = 67.0;
    public final static double WRIST_SERVO_BASKET1 = 0.600;     // AUTO/TELE: lifting arm toward basket
    public final static double WRIST_SERVO_BASKET1_ANGLE = 157.0;
    public final static double WRIST_SERVO_RAISE = 0.570;
    public final static double WRIST_SERVO_RAISE_ANGLE = 157.0;
    public final static double WRIST_SERVO_BASKET2 = 0.220;     // AUTO/TELE: scoring in basket
    public final static double WRIST_SERVO_BASKET2_ANGLE = 270.0;
    public final static double WRIST_SERVO_BAR1 = 0.640;         // AUTO: specimen bar (when above)
    public final static double WRIST_SERVO_BAR1_ANGLE = 173.0;
    public final static double WRIST_SERVO_BAR2 = 0.640;         // AUTO: specimen bar (when clipped)
    public final static double WRIST_SERVO_BAR2_ANGLE = 173.0;
    public final static double WRIST_SERVO_WALL0 = 0.500;        // AUTO: grab specimen off wall (on approach)
    public final static double WRIST_SERVO_WALL0_ANGLE = 180.0;
    public final static double WRIST_SERVO_WALL1 = 0.519;        // AUTO: grab specimen off wall (lift off)
    public final static double WRIST_SERVO_WALL1_ANGLE = 173.0;
    public final static double WRIST_SERVO_CLIP = 0.350;        // AAUTO: clip specimen on bar by just driving forward

    // horizontal = 0.440
    // straight down = 0.710

    //public CRServo geckoServo = null;

    //===== Claw servo =====
    public Servo clawServo = null;

    public final static double CLAW_SERVO_CLOSED  = 0.443;  // Claw closed (hold sample/specimen)
    public final static double CLAW_SERVO_INIT    = 0.500;  // Claw in init position (servo default power-on state)
    public final static double CLAW_SERVO_OPEN_N  = 0.600;  // claw opened narrow (enough to release/drop)
    public final static double CLAW_SERVO_OPEN_W  = 0.850;  // claw opened wide (fully open and above samples on floor)

    public enum clawStateEnum {
        CLAW_INIT,
        CLAW_OPEN_NARROW,
        CLAW_OPEN_WIDE,
        CLAW_OPEN,       /* used to toggle between OPEN_NARROW and OPEN_WIDE */
        CLAW_CLOSED
    }

    public HardwareMinibot.clawStateEnum clawState = HardwareMinibot.clawStateEnum.CLAW_INIT;

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
//      odom.setOffsets(0.0, -48.0);     // odometry pod x,y locations relative center of robot  NOW
//      odom.setOffsets(0.0, 0.0);       // odometry pod x,y locations relative center of robot  REFERENCE
        odom.setOffsets(-144.00, +88.00); // odometry pod x,y locations relative center of robot  2 2
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

        elbowServo    = hwMap.servo.get("ElbowServo");          // servo port 0 (Expansion Hub)
        elbowServoPos = hwMap.analogInput.get("ElbowServoPos"); // Analog port 1 (Expansion Hub)

        wristServo    = hwMap.servo.get("WristServo");          // servo port 1 (Expansion Hub)
        wristServoPos = hwMap.analogInput.get("WristServoPos"); // Analog port 0 (Expansion Hub)

        clawServo     = hwMap.servo.get("ClawServo");           // servo port 2 (Expansion Hub)

        if( isAutonomous ) {
            elbowServo.setPosition(ELBOW_SERVO_INIT);
            wristServo.setPosition(WRIST_SERVO_INIT);
            clawServo.setPosition(CLAW_SERVO_INIT);
        }

//      geckoServo = hwMap.crservo.get("GeckoServo");           // servo port 2 (Expansion Hub)
//      geckoServo.setPower(0.0);

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

        elbowServo.setPosition(ELBOW_SERVO_INIT);
        wristServo.setPosition(WRIST_SERVO_INIT);
        clawServo.setPosition(CLAW_SERVO_INIT);

    } // resetEncoders

    /*--------------------------------------------------------------------------------------------*/
    public void clawStateSet( HardwareMinibot.clawStateEnum newClawState )
    {
        switch( newClawState ) {
            case CLAW_INIT :
                clawServo.setPosition( CLAW_SERVO_INIT );
                clawState = newClawState;
                break;
            case CLAW_OPEN :  // OPEN is used to toggle between OPEN_NARROW and OPEN_WIDE
                if( clawState == HardwareMinibot.clawStateEnum.CLAW_OPEN_NARROW ) {
                    clawServo.setPosition( CLAW_SERVO_OPEN_W );
                    clawState = HardwareMinibot.clawStateEnum.CLAW_OPEN_WIDE;
                } else if( clawState == HardwareMinibot.clawStateEnum.CLAW_OPEN_WIDE ) {
                    clawServo.setPosition( CLAW_SERVO_OPEN_N );
                    clawState = HardwareMinibot.clawStateEnum.CLAW_OPEN_NARROW;
                } else { // Not currently OPEN in either NARROW or WIDE; start NARROW
                    clawServo.setPosition( CLAW_SERVO_OPEN_N );
                    clawState = HardwareMinibot.clawStateEnum.CLAW_OPEN_NARROW;
                }
                break;
            case CLAW_OPEN_NARROW :
                clawServo.setPosition( CLAW_SERVO_OPEN_N );
                clawState = newClawState;
                break;
            case CLAW_OPEN_WIDE :
                clawServo.setPosition( CLAW_SERVO_OPEN_W );
                clawState = newClawState;
                break;
            case CLAW_CLOSED :
                clawServo.setPosition( CLAW_SERVO_CLOSED );
                clawState = newClawState;
                break;
            default:
                break;
        } // switch()

    } // clawStateSet

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

    public void startWormTilt(double targetArmAngle)
    {   // Convert angle to encoder counts
        int targetEncoderCount = computeEncoderCountsFromAngle(targetArmAngle);
        // Range-check the target
        if( targetArmAngle < TILT_ANGLE_HW_MIN_DEG) targetEncoderCount = computeEncoderCountsFromAngle(TILT_ANGLE_HW_MIN_DEG);
        if( targetArmAngle > TILT_ANGLE_HW_MAX_DEG ) targetEncoderCount = computeEncoderCountsFromAngle(TILT_ANGLE_HW_MAX_DEG);

        wormTiltMotor.setTargetPosition( targetEncoderCount );
        wormTiltMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

        wormTiltMotor.setPower(0.8);
        wormTiltTimer.reset();
        wormTiltMotorAutoMove = true;
        wormTiltMotorBusy = true;
    } // startWormTilt
    public void processWormTilt(){
        // Has the automatic movement reached its destination?.
        if( wormTiltMotorAutoMove ) {
            if (!wormTiltMotor.isBusy()) {
                wormTiltMotorBusy = false;
                // Timeout reaching destination.
            } else if (wormTiltTimer.milliseconds() > 5000) {
                wormTiltMotorBusy = false;
                //telemetry.addData("processViperSlideExtension", "Movement timed out.");
                //telemetry.addData("processViperSlideExtension", "Position: %d", viperMotor.getCurrentPosition());
                //telemetry.update();
                //telemetrySleep();
            }
        }
    } // processWormTilt

    public void abortWormTilt()
    {
        // Have we commanded an AUTOMATIC lift movement that we need to terminate so we
        // can return to MANUAL control?  (NOTE: we don't care here whether the AUTOMATIC
        // movement has finished yet; MANUAL control immediately overrides that action.
        if( wormTiltMotorAutoMove ) {
            // turn off the auto-movement power, but don't go to ZERO POWER or
            // the weight of the lift will immediately drop it back down.
            wormTiltMotor.setMode(  DcMotor.RunMode.RUN_USING_ENCODER );
            wormTiltMotor.setPower( 0.0 );
//         liftMoveState = LiftMoveActivity.IDLE;
//         liftStoreState = LiftStoreActivity.IDLE;
            wormTiltMotorAutoMove = false;
            wormTiltMotorBusy = false;
        }
    } // abortWormTilt

    public void startViperSlideExtension(int targetEncoderCount )
    {
        startViperSlideExtension(targetEncoderCount, VIPER_RAISE_POWER, -VIPER_LOWER_POWER);
    } // startViperSlideExtension

    public void startViperSlideExtension(int targetEncoderCount, double raisePower, double lowerPower)
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
        double motorPower = (directionUpward)? raisePower : lowerPower;
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
