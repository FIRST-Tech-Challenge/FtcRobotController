package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


public class RobotHardware {
    // The IMU sensor object
    BNO055IMU imu;  //gyro built into the rev hub

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    double GlobalHeading = 0, LastHeading = 0;

    VoltageSensor battery;

    /* Declare OpMode members. */
    private OpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor wheel1 = null;
    private DcMotor wheel2 = null;
    private DcMotor wheel3 = null;
    private DcMotor wheel4 = null;

    public DcMotor slide = null;
    public DcMotor liftL = null;
    public DcMotor liftR = null;
    public DcMotor slidewrist = null;
    public Servo spinner = null;
    public Servo claw = null;

    //odometers
    public DcMotor encoderLeft;
    public DcMotor encoderRight;
    public DcMotor encoderAux;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime moveTimer = new ElapsedTime();

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(OpMode opmode) {myOpMode = opmode; }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {
        BNO055IMU.Parameters Gparameters = new BNO055IMU.Parameters();
        Gparameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        Gparameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Gparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode

        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(Gparameters);

        myOpMode.telemetry.addData("Gyro Status", "Initialized");
        myOpMode.telemetry.update();

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        battery = myOpMode.hardwareMap.get(VoltageSensor.class, "Control Hub");
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        wheel1  = myOpMode.hardwareMap.get(DcMotor.class, "wheel1");
        wheel2  = myOpMode.hardwareMap.get(DcMotor.class, "wheel2");
        wheel3  = myOpMode.hardwareMap.get(DcMotor.class, "wheel3");
        wheel4  = myOpMode.hardwareMap.get(DcMotor.class, "wheel4");

        spinner = myOpMode.hardwareMap.get(Servo.class, "spinner");
        claw = myOpMode.hardwareMap.get(Servo.class, "claw");

        slide  = myOpMode.hardwareMap.get(DcMotor.class, "slide");
        slidewrist  = myOpMode.hardwareMap.get(DcMotor.class, "slidewrist");
        liftL = myOpMode.hardwareMap.get(DcMotor.class, "liftL");
        liftR = myOpMode.hardwareMap.get(DcMotor.class, "liftR");

        slidewrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setDirection(DcMotor.Direction.FORWARD);
        //lift.setDirection(DcMotor.Direction.REVERSE);
        slidewrist.setDirection(DcMotor.Direction.REVERSE);


        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidewrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidewrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wheel1.setDirection(DcMotor.Direction.REVERSE);
        wheel2.setDirection(DcMotor.Direction.REVERSE);
        wheel3.setDirection(DcMotor.Direction.FORWARD);
        wheel4.setDirection(DcMotor.Direction.FORWARD);

        wheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheel3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheel4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //shadow the motors with the odo encoders
        encoderLeft = wheel1;
        encoderRight = wheel2;
        encoderAux = wheel3;


        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }
    public void resetDriveEncoders() {
        wheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // constants that define the geometry of the robot
    final static double L = 21.5;
    final static double B = -5.24;
    final static double R = 2.0;
    final static double N = 8192;
    final static double cm_per_tick = 2.0 * Math.PI * R / N * 80/83.8;
    final static double inch_per_cm = 0.3937;
    final static double xWheelOffset = 0.0;



    public void driveRobot(double Drive, double Side, double Spin) {
        // Combine drive and turn for blended motion.
        double wheel1Pwr  = Drive - Side - Spin; // front right
        double wheel2Pwr  = Drive + Side - Spin; // back right
        double wheel3Pwr  = Drive - Side + Spin; // back left
        double wheel4Pwr  = Drive + Side + Spin; // front left

        // Scale the values so none exceed +/- 1.0
        double max = Math.max(Math.abs(wheel1Pwr), Math.abs(wheel2Pwr));
        max = Math.max(Math.abs(wheel3Pwr), Math.abs(max));
        max = Math.max(Math.abs(wheel4Pwr), Math.abs(max));
        if (max > 1.0)
        {
            wheel1Pwr /= max;
            wheel2Pwr /= max;
            wheel3Pwr /= max;
            wheel4Pwr /= max;
        }

        // Use existing function to drive both wheels.
        setDrivePower(wheel1Pwr, wheel2Pwr, wheel3Pwr, wheel4Pwr);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     */
    public void setDrivePower(double wheel1p, double wheel2p, double wheel3p, double wheel4p) {
        // Output the values to the motor drives.
        wheel1.setPower(wheel1p);
        wheel2.setPower(wheel2p);
        wheel3.setPower(wheel3p);
        wheel4.setPower(wheel4p);
    }


    public double GetHeading()
    {
        double deltaHeading = 0;
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        deltaHeading = angles.firstAngle - LastHeading;
        if (deltaHeading < -180) {
            deltaHeading = deltaHeading + 360;
        } else if (deltaHeading > 180) {
            deltaHeading = deltaHeading - 360;
        }
        GlobalHeading = GlobalHeading + deltaHeading;
        LastHeading = angles.firstAngle;

        return GlobalHeading;
    }
} // whole file