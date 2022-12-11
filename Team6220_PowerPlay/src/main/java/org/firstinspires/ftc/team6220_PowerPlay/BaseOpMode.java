package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class BaseOpMode extends LinearOpMode {

    // motors
    public static DcMotorEx motorFL;
    public static DcMotorEx motorFR;
    public static DcMotorEx motorBL;
    public static DcMotorEx motorBR;

    public static DcMotorEx motorTurntable;
    public static DcMotorEx motorLeftSlides;
    public static DcMotorEx motorRightSlides;

    // servos
    public static Servo servoGrabber;

    // IMU
    public BNO055IMU imu;
    public Orientation IMUOriginalAngles; // original angle reading from imu that will be used to find unwanted angle offset during drive

    // flag to say whether we should disable the correction system
    private boolean turnFlag = false;

    // initializes the motors, servos, and IMUs
    public void initialize() {
        // motors
        motorFL = (DcMotorEx) hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = (DcMotorEx) hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = (DcMotorEx) hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = (DcMotorEx) hardwareMap.get(DcMotor.class, "motorBR");
        motorTurntable = (DcMotorEx) hardwareMap.get(DcMotor.class, "motorTurntable");
        motorLeftSlides = (DcMotorEx) hardwareMap.get(DcMotor.class, "motorLVSlides");
        motorRightSlides = (DcMotorEx) hardwareMap.get(DcMotor.class, "motorRVSlides");

        motorFL.setDirection(DcMotorEx.Direction.FORWARD);
        motorFR.setDirection(DcMotorEx.Direction.REVERSE);
        motorBL.setDirection(DcMotorEx.Direction.FORWARD);
        motorBR.setDirection(DcMotorEx.Direction.REVERSE);

        motorFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorTurntable.setDirection(DcMotorEx.Direction.FORWARD);
        motorLeftSlides.setDirection(DcMotorEx.Direction.FORWARD);
        motorRightSlides.setDirection(DcMotorEx.Direction.REVERSE);

        motorTurntable.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRightSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorTurntable.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftSlides.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorRightSlides.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motorTurntable.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLeftSlides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRightSlides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // servos
        servoGrabber = hardwareMap.servo.get("servoGrabber");

        // initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        servoGrabber.setPosition(Constants.GRABBER_INITIALIZE_POSITION);
    }

    public void driveWithIMU(double xPower, double yPower, double tPower) {
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

    /**
     * this method will allow the grabber to open or close given a boolean input
     * @param position the position of the grabber
     */
    public void driveGrabber(double position) {
        servoGrabber.setPosition(position);
    }

    /**
     * this method will allow the slides to move given a target position
     * @param targetPosition target position for slides motors in ticks
     */
    public void driveSlides(int targetPosition) {
        int error = targetPosition - motorLeftSlides.getCurrentPosition();
        double motorPower = Math.max(error * 0.005, 0.2);

        // slides not yet at target position
        if (Math.abs(error) > 20) {
            // slides going down - constant speed
            if (error < 0) {
                motorLeftSlides.setPower(-0.75);
                motorRightSlides.setPower(-0.75);
            // slides going up - proportional control
            } else {
                motorLeftSlides.setPower(motorPower);
                motorRightSlides.setPower(motorPower);
            }
        // slides at target position
        } else {
            motorLeftSlides.setPower(0.05);
            motorRightSlides.setPower(0.05);
        }
    }

    /**
     * this method will allow the turntable to turn given a target position
     * @param targetPosition target position for turntable motor in ticks
     */
    public void driveTurntable(double targetPosition) {

    }
}
