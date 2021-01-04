package org.firstinspires.ftc.teamcode.team10515;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.team10515.subsystems.ForkliftSubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.FlywheelSubsystem;

public class UGMapTest
{

    public DcMotor Shooter1 = null;  //FrontRight
    public DcMotor Shooter2 = null;  //FrontLeft
//    public DcMotor RR = null;  //RearRight
//    public DcMotor RL = null;  //RearLeft
//    public DcMotor LL = null;  //LiftLeft
//    public DcMotor LR = null;  //LiftRight
//    public DcMotor INR = null;  //Intake
//    public DcMotor INL = null;  //Intake
    //public DcMotor  AA = null;  //AccessArm
    public DcMotor forkliftMotor = null; //Wobble goal
    public DcMotor intakeMotor = null;

    public BNO055IMU imu = null;


    static private final String SHOOTER_1 = "Shooter 1";
    static private final String SHOOTER_2 = "Shooter 2";
//    static private final String REARRIGHT      = "RR";
//    static private final String REARLEFT       = "RL";
    static private final String FORKLIFTMOTOR  = "Forklift Motor";
    static private final String INTAKEMOTOR = "Intake Motor";


    //static private final String  DEPOSITLIFT    = "DL";

//    static final String  IMU_SENSOR = "imu";

    private ShooterSubsystem shooterMotors;
    private ForkliftSubsystem forkliftMotorSub;
    private FlywheelSubsystem intakeMotorSub;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public UGMapTest() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        Shooter2 = hwMap.dcMotor.get(SHOOTER_2);
        Shooter1 = hwMap.dcMotor.get(SHOOTER_1);
//        RL      = hwMap.dcMotor.get(REARLEFT);
//        RR      = hwMap.dcMotor.get(REARRIGHT);

        //AA      = hwMap.dcMotor.get(ACCESSARM);

//        forkliftServo = hwMap.servo.get(FORKLIFTSERVO);

        intakeMotor = hwMap.dcMotor.get(INTAKEMOTOR);
        forkliftMotor = hwMap.dcMotor.get(FORKLIFTMOTOR);

        Shooter2.setDirection(DcMotor.Direction.REVERSE);
        Shooter1.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        forkliftMotor.setDirection(DcMotor.Direction.FORWARD);
//        RL.setDirection(DcMotor.Direction.REVERSE);
//        RR.setDirection(DcMotor.Direction.FORWARD);
        //AA.setDirection(DcMotor.Direction.REVERSE);



        //set motor power behaviour
        Shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        forkliftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //AA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forkliftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // TODO: Change to RUN_WITH_ENCODER
        forkliftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //INL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

         //Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
//        imu = hwMap.get(BNO055IMU.class, IMU_SENSOR);
//        imu.initialize(parameters);

    }
    public ShooterSubsystem getShooter() { return shooterMotors; }

    public ForkliftSubsystem getForkliftMotor() {
        return forkliftMotorSub;
    }

    public void setShooterMotors(ShooterSubsystem shooterMotors){ this.shooterMotors = shooterMotors; }

    public FlywheelSubsystem getIntakeMotorSub(){
        return intakeMotorSub;
    }
}
