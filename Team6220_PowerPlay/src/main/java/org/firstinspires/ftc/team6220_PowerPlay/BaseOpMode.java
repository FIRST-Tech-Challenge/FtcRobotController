package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class BaseOpMode extends LinearOpMode {

    // motors
    public static DcMotor motorFL;
    public static DcMotor motorFR;
    public static DcMotor motorBL;
    public static DcMotor motorBR;

    public static DcMotor motorTurntable;
    public static DcMotor motorLVSlides;
    public static DcMotor motorRVSlides;

    // servos
    public static Servo servoGrabber;

    // IMU
    public BNO055IMU imu;
    public Orientation IMUOriginalAngles; // original angle reading from imu that will be used to find unwanted angle offset during drive

    // flag to say whether we should disable the correction system
    private boolean turnFlag = false;

    private double error;
    private double ySlidePower;

    // initializes the motors, servos, and IMUs
    public void initialize() {

        // motors
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorTurntable = hardwareMap.dcMotor.get("motorTurntable");
        motorLVSlides = hardwareMap.dcMotor.get("motorLVSlides");
        motorRVSlides = hardwareMap.dcMotor.get("motorRVSlides");

        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorLVSlides.setDirection(DcMotor.Direction.FORWARD);
        motorRVSlides.setDirection(DcMotor.Direction.REVERSE);
        motorTurntable.setDirection(DcMotor.Direction.FORWARD);

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorTurntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLVSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRVSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorTurntable.setTargetPosition(0);
        motorTurntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLVSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRVSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // servos
        servoGrabber = hardwareMap.servo.get("servoGrabber");

        // initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // preset the IMU angles so it doesn't start on null since it will only later be read when turning
        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        servoGrabber.setPosition(Constants.GRABBER_INITIALIZE_POSITION);
    }

    public void driveWithIMU(double xPower, double yPower, double tPower)  {

        // read imu when turning (when t != 0)
        boolean isTurning = (tPower != 0);

        if (isTurning || turnFlag) {
            IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); // set original angle

            if (!turnFlag) {
                turnFlag = true;
            }

        // otherwise read imu for correction
        } else {
            // obtain the current angle's error from the original angle
            Orientation currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double angleError = IMUOriginalAngles.firstAngle - currentAngle.firstAngle;

            // flip to inverse of angles above 180 / below -180 (to prevent infinity-rotate bug)
            // to make sure to use the shorter angle
            if (angleError > 180.0) {
                angleError -= 360.0;
            } else if (angleError < -180.0) {
                angleError += 360.0;
            }

            // apply a constant to turn the angle into a turn speed
            tPower = -Constants.CORRECTION_CONSTANT * angleError;
        }

        // if the rotation rate is low, then that means all the momentum has left the robot's turning and can therefore turn the correction back on
        if (Math.abs(imu.getAngularVelocity().zRotationRate) < 5) {
            turnFlag = false;
        }

        // calculate speed and direction of each individual motor and set power of motors to speed
        motorFL.setPower(-yPower + xPower + tPower);
        motorFR.setPower(-yPower - xPower - tPower);
        motorBL.setPower(-yPower - xPower + tPower);
        motorBR.setPower(-yPower + xPower - tPower);
    }

    // this method will allow the grabber to open or close given a boolean input, with true = open and false = close
    public void driveGrabber(boolean isOpen) {
        if (isOpen) {
            servoGrabber.setPosition(Constants.GRABBER_OPEN_POSITION); // set servo to open position
        } else {
            servoGrabber.setPosition(Constants.GRABBER_CLOSE_POSITION); // set servo to closed position
        }
    }

    // this method will allow the slides to move upwards, downwards, outwards, and inwards given a specified x target position and y target position
    public void driveSlides(/*int xTargetPosition,*/ int yTargetPosition) {
        error = yTargetPosition - motorLVSlides.getCurrentPosition();
        ySlidePower = Math.max(error * Constants.VERTICAL_SLIDE_P_CONSTANT, 0.25);

        motorLVSlides.setPower(ySlidePower);
        motorRVSlides.setPower(ySlidePower);
    }

    // this method will allow the turntable to turn clockwise or counterclockwise given a specified power and position
    public void driveTurntable(double power, int position) {
        motorTurntable.setPower(power);
        motorTurntable.setTargetPosition(position);
    }
}
