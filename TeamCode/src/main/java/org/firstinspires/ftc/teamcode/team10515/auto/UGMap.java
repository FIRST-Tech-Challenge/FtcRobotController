package org.firstinspires.ftc.teamcode.team10515.auto;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class UGMap
{

    public DcMotor FR = null;  //FrontRight
    public DcMotor FL = null;  //FrontLeft
    public DcMotor RR = null;  //RearRight
    public DcMotor RL = null;  //RearLeft
    public DcMotor LL = null;  //LiftLeft
    public DcMotor LR = null;  //LiftRight
    public DcMotor INR = null;  //Intake
    public DcMotor INL = null;  //Intake
    //public DcMotor  AA = null;  //AccessArm


    public BNO055IMU imu = null;


    static private final String FRONTRIGHT     = "FR";
    static private final String FRONTLEFT      = "FL";
    static private final String REARRIGHT      = "RR";
    static private final String REARLEFT       = "RL";


    //static private final String  DEPOSITLIFT    = "DL";

    static final String  IMU_SENSOR = "imu";


    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public UGMap() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        FL      = hwMap.dcMotor.get(FRONTLEFT);
        FR      = hwMap.dcMotor.get(FRONTRIGHT);
        RL      = hwMap.dcMotor.get(REARLEFT);
        RR      = hwMap.dcMotor.get(REARRIGHT);

        //AA      = hwMap.dcMotor.get(ACCESSARM);



        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        RL.setDirection(DcMotor.Direction.REVERSE);
        RR.setDirection(DcMotor.Direction.FORWARD);
        //AA.setDirection(DcMotor.Direction.REVERSE);



        //set motor power behaviour
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //AA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // May want to use RUN_USING_ENCODERS if encoders are installed.
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //INL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

         //Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, IMU_SENSOR);
        imu.initialize(parameters);

    }
}
