package org.firstinspires.ftc.team417_2021;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team417_2021.Resources.FIRFilter;
import org.firstinspires.ftc.team417_2021.Resources.PIDFilter;
import org.firstinspires.ftc.team417_2021.Resources.Polynomial;
import com.qualcomm.robotcore.hardware.Servo;

abstract public class MasterOpMode extends LinearOpMode {
    DcMotor motorFL = null;
    DcMotor motorFR = null;
    DcMotor motorBL = null;
    DcMotor motorBR = null;

    DcMotor shoulderMotor = null;
    DcMotor elbowMotor = null;
    Servo wristServo = null;
    Servo grabberServo = null;
    DcMotor carouselMotor = null;

    public Robot robot = new Robot(this);

    public BNO055IMU imu;

    PIDFilter turnFilter;
    PIDFilter moveFilter;
    FIRFilter accelerationFilter;
    int filterLength = 10;

    // drive constants
    static final double COUNTS_PER_MOTOR_REV = 537.7; // GoBilda 5302 19:2:1 motor
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // arm and grabber constants
    static final double GRABBER_IN = 0.9;
    static final double GRABBER_OUT = 0.0;
    static final int ELBOW_LEVEL_1 = 1460;
    static final int ELBOW_LEVEL_2 = 1430;
    static final int ELBOW_LEVEL_3 = 300;
    static final int ELBOW_CAP = 0;
    static final int SHOULDER_LEVEL_1 = -1770;
    static final int SHOULDER_LEVEL_2 = -1650;
    static final int SHOULDER_LEVEL_3 = -1500;
    static final int SHOULDER_CAP = 0; // todo get correct values
    static final double WRIST_POS = 0.4;

    protected void initializeHardware() {
        // initialize move filters
        turnFilter = new PIDFilter(0.008, 0, 0.0005);
        moveFilter = new PIDFilter(0.04, 0, 0);
        // weights for weighted average
        double[] filterCoefficients = {1};
        accelerationFilter = new FIRFilter(new Polynomial(filterCoefficients),filterLength);
        accelerationFilter.values = new double[filterLength];

        // Initialize motors to be the hardware motors
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        shoulderMotor = hardwareMap.dcMotor.get("shoulderMotor");
        elbowMotor = hardwareMap.dcMotor.get("elbowMotor");
        carouselMotor = hardwareMap.dcMotor.get("carouselMotor");

        wristServo = hardwareMap.servo.get("wristServo");
        grabberServo = hardwareMap.servo.get("grabberServo");

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD); // todo reverse back

        // set motor power to 0
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

        shoulderMotor.setPower(0);
        elbowMotor.setPower(0);
        wristServo.setPosition(WRIST_POS);
        carouselMotor.setPower(0.0);
        grabberServo.setPosition(GRABBER_IN);

        // set up IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "Adaf'" +
                "ruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.log().setCapacity(8);

    }

    public void drive(double drivePower, double rotationalPower) {

        double frontLeft = drivePower + rotationalPower;
        double frontRight = drivePower - rotationalPower;
        double backLeft = drivePower + rotationalPower;
        double backRight = drivePower - rotationalPower;

        motorFL.setPower(frontLeft);
        motorFR.setPower(frontRight);
        motorBL.setPower(backLeft);
        motorBR.setPower(backRight);
    }

    public void runMotorToPosition(DcMotor motor, int targetPosition, double power) {

        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);


    }

}
