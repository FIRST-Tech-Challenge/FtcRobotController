package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    //public static DcMotor motorTurntable;
    //public static DcMotor motorLVSlides;
    //public static DcMotor motorRVSlides;

    // servos
    //public static Servo servoGrabber;

    // IMU
    public BNO055IMU imu;
    Orientation IMUOriginalAngles; // original angle reading from imu that will be used to find unwanted angle offset during drive
    double correctionConstant = 1 / 45.0; // constant for converting angle error to motor speed

    // other values
    double driveSpeedModifier = 0.5;

    // initializes the motors, servos, and IMUs
    public void initialize() {

        // motors
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        //motorTurntable = hardwareMap.dcMotor.get("motorTurntable");
        //motorLVSlides = hardwareMap.dcMotor.get("motorLVSlides");
        //motorRVSlides = hardwareMap.dcMotor.get("motorRVSlides");

        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //motorTurntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorLVSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorRVSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //motorTurntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //motorLVSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //motorRVSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // servos
        //servoGrabber = hardwareMap.servo.get("servoGrabber");

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
    }

    // this method drives holonomic when given a horizontal power, vertical power, and turn power
    public void driveHolo(double xPower, double yPower, double tPower) {
        double speedFL = (xPower - yPower + tPower);
        double speedFR = (-xPower - yPower - tPower);
        double speedBL = (-xPower - yPower + tPower);
        double speedBR = (xPower - yPower - tPower);
    }

    // flag to say whether we should disable the correction system
    boolean turnFlag = false;

    public void driveRobot(double xPower, double yPower, double tPower)  {

        telemetry.addData("org: ", IMUOriginalAngles.firstAngle); //temp
        telemetry.addData("t1=", tPower); //temp
        // read imu when turning (when t != 0)

        boolean isTurning = tPower != 0; // should we disable the correction based on whether the robot is turning because of user input

        if (isTurning || turnFlag) {
            IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); // set original angle

            // if the robot is turning because of user input and the disable flag is false
            if (!turnFlag) {
                turnFlag = true; // set to true
            }

        // otherwise read imu for correction
        } else {
            // obtain the current angle's error from the original angle
            Orientation currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double angleError = IMUOriginalAngles.firstAngle - currentAngle.firstAngle;

            // flip to inverse of angles above 180/below -180 (to prevent funny infinirotate bug)
            // to make sure to use the shorter angle
            if (angleError > 180.0) {
                angleError -= 360.0;
            } else if (angleError < -180.0) {
                angleError += 360.0;
            }

            telemetry.addData("error=", angleError); // temp

            // apply a constant to turn the angle into a turn speed
            tPower = -correctionConstant * angleError;
        }

        // if the rotation rate is low, then that means all the momentum has left the robot's turning and can therefore turn the correction back on
        if (Math.abs(imu.getAngularVelocity().zRotationRate) < 5) {
            turnFlag = false;
        }

        telemetry.addData("t2=", tPower); // temp
        telemetry.addLine("hello, this is cool!"); // temp
        telemetry.update(); // temp

        // calculate speed and direction of each individual motor
        double speedFL = (-yPower + xPower + tPower) * driveSpeedModifier;
        double speedFR = (-yPower - xPower - tPower) * driveSpeedModifier;
        double speedBL = (-yPower - xPower + tPower) * driveSpeedModifier;
        double speedBR = (-yPower + xPower - tPower) * driveSpeedModifier;

        // set power of motors to speed
        motorFL.setPower(speedFL);
        motorFR.setPower(speedFR);
        motorBL.setPower(speedBL);
        motorBR.setPower(speedBR);

        telemetry.addData("IMU disabled flag", turnFlag);
        telemetry.addData("z rot vel", imu.getAngularVelocity().zRotationRate);
    }

    // method to open/close grabber if you set boolean to true or false
    public void openGrabber(boolean open) {
        if (open) {
            //servoGrabber.setPosition(0.33); // set servo to open position
        } else {
            //servoGrabber.setPosition(0.11); // set servo to closed position
        }
    }

    // this is a general method to turn the turntable
    public void rotateTurntable(double power) {
        //motorTurntable.setPower(power);
    }
}
