package org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;

@Config
public abstract class BaseOpMode extends LinearOpMode {
    //Declares LEDs on DevBot
    public DigitalChannel red;
    public DigitalChannel green;

    //Declares drive-motors
    public DcMotor FR;
    public DcMotor FL ;
    public DcMotor BR ;
    public DcMotor BL ;

    public DcMotor intakeMotor ;
    public DcMotor armMotor ;
    static final public double ARM_MOTOR_MIN_POSITION = 0;
    static final public double ARM_MOTOR_MAX_POSITION = 4200;
    public Servo dumperServo;
    public static final double DUMPER_SERVO_TILT_POSITION = 0.4;
    public static final double DUMPER_SERVO_RESET_POSITION = 0.527;
    public static final double DUMPER_SERVO_DUMP_POSITION = 0.2;
    public Servo gateServo;
    public final double GATE_SERVO_OPEN_POSITION = 0;
    public final double GATE_SERVO_CLOSE_POSITION = 0.55;

    public static final double TICKS_PER_REVOLUTION = 537.7 * (24.0/27); // 5203 Series Yellow Jacket Motor, robot was overshooting so
    public static final double GEAR_RATIO = 1.0;
    public static final double WHEEL_DIAMETER = 3.7; // inches
    public static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);
    public static final double INCHES_PER_REVOLUTION = Math.PI * WHEEL_DIAMETER;
    public static final double INCHES_PER_TICK = INCHES_PER_REVOLUTION / TICKS_PER_REVOLUTION;

    //Declares IMU
    //public BNO055IMU imu;

    //Initializes motors, servos, and sensors
    public void initializeHardware() {
        //Drive Motors, other motors, sensors, etc.
        if(MecanumDrive.isDevBot) {
            FL = initializeMotor("leftFront", DcMotor.Direction.REVERSE);
            FR = initializeMotor("rightFront", DcMotor.Direction.FORWARD);
            BL = initializeMotor("leftBack", DcMotor.Direction.REVERSE);
            BR = initializeMotor("rightBack", DcMotor.Direction.FORWARD);

            red = initializeDigitalChannel("red", DigitalChannel.Mode.OUTPUT);
            green = initializeDigitalChannel("green", DigitalChannel.Mode.OUTPUT);

            // Turn LEDs off (counterintuitive, on = false, off = true)
            red.setState(true);
            green.setState(true);
        } else {
            FL = initializeMotor("FLMotor", DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FR = initializeMotor("FRMotor", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BL = initializeMotor("BLMotor", DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BR = initializeMotor("BRMotor", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //Mechanism Motors
            intakeMotor = initializeMotor("IntakeMotor", DcMotor.Direction.FORWARD);
            armMotor = initializeMotor("ArmMotor", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);//DcMotor.RunMode.RUN_TO_POSITION);

            //Mechanism Servos
            dumperServo = initializeServo("DumperServo", Servo.Direction.FORWARD);
            gateServo = initializeServo("GateServo", Servo.Direction.FORWARD);
        }

        //In case we ever need an IMU
        /*
        // Sets up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample op-mode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        // Retrieves and initializes the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Waits so the imu can process
        sleep(2000);
        */
    }

    public DigitalChannel initializeDigitalChannel(String channelName, DigitalChannel.Mode mode) {
        DigitalChannel digitalChannel = hardwareMap.get(DigitalChannel.class, channelName);
        digitalChannel.setMode(mode);
        return digitalChannel;
    }

    public DcMotor initializeMotor(String motorName, DcMotorSimple.Direction direction, DcMotor.RunMode mode) {
        DcMotor motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(mode);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
        return motor;
    }

    public DcMotor initializeMotor(String motorName, DcMotor.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
        return motor;
    }

    public Servo initializeServo(String servoName, Servo.Direction direction) {
        Servo servo = hardwareMap.get(Servo.class, servoName);
        servo.setDirection(direction);
        return servo;
    }

    public void mecanumDrive(double x, double y, double rot) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rot), 1);

        double frontLeftPower = (y + x + rot) / denominator;
        double frontRightPower = (y - x - rot) / denominator;
        double backLeftPower = (y - x + rot) / denominator;
        double backRightPower = (y + x - rot) / denominator;

        FL.setPower(frontLeftPower);
        FR.setPower(frontRightPower);
        BL.setPower(backLeftPower);
        BR.setPower(backRightPower);
    }

    public void runIntakeMechanism(double speed) {
        intakeMotor.setPower(speed);
    }



    public void tiltDumper() {
        dumperServo.setPosition(DUMPER_SERVO_TILT_POSITION);
    }

    public void resetDumper() {
        dumperServo.setPosition(DUMPER_SERVO_RESET_POSITION);
    }

    public void dumpDumper() {
        dumperServo.setPosition(DUMPER_SERVO_DUMP_POSITION);
    }

    enum DumperAction {
        DUMPING,
        RESETTING,
        STOPPING
    }
    public void moveDumper(DumperAction dumperAction) {
        switch (dumperAction) {
            case DUMPING:
                dumperServo.setPosition(1);
            case RESETTING:
                dumperServo.setPosition(0);
            case STOPPING:
                dumperServo.setPosition(dumperServo.getPosition());
            default:
        }
    }

    public void openGate() {
        gateServo.setPosition(GATE_SERVO_OPEN_POSITION);
    }

    public void closeGate() {
        gateServo.setPosition(GATE_SERVO_CLOSE_POSITION);
    }

    public double[] armPositions = new double[] {ARM_MOTOR_MIN_POSITION, ARM_MOTOR_MIN_POSITION + ((ARM_MOTOR_MAX_POSITION - ARM_MOTOR_MIN_POSITION) / 2), ARM_MOTOR_MAX_POSITION};

    public void moveArm(double speed) {
        if (armMotor.getCurrentPosition() > ARM_MOTOR_MAX_POSITION) {
            speed = -0.1;
        } else if (armMotor.getCurrentPosition() < ARM_MOTOR_MIN_POSITION) {
            speed = 0.1;
        }
        armMotor.setPower(speed);
    }

    public final static double EPSILON = 0.0001;
    public static boolean isEpsilonEquals(double a, double b) {
        return (Math.abs(a) + EPSILON >= Math.abs(b) && Math.abs(a) - EPSILON <= Math.abs(b));
    }
}
