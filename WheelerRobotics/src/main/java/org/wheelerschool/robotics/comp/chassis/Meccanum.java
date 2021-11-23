package org.wheelerschool.robotics.comp.chassis;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Meccanum {

    private ElapsedTime runtime = new ElapsedTime();

    private Servo servo0;
    private DcMotor arm;

    public final double SERVO_FULLY_CLOSED = 0;
    public final double SERVO_FULLY_OPENED = 0;
    public final double ARM_MAX_SPEED = 0;
    public final double HIGH_SPINNER_POWER = 0;
    public final double NORMAL_SPEED = 0;
    public final double OPTIMAL_SPINNER_POWER = 0;
    public final double MOTOR_STOP = 0;

    private DcMotor spinner;

    private BNO055IMU imu;

    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;

    private Orientation angles;

    public void init(HardwareMap hardwareMap){

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        // Meccanum Motors Definition and setting prefs

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        // Reverse the left side motors and set behaviors to stop instead of coast
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //define arm and servo objects and also spinner
        servo0 = hardwareMap.get(Servo.class, "servo-0");
        arm = hardwareMap.get(DcMotor.class, "arm");
        spinner = hardwareMap.get(DcMotor.class, "spinner");

        //set prefs for arm and servo
        servo0.setDirection(Servo.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner = hardwareMap.get(DcMotor.class, "spinner");


        runtime.reset();

    }


    public void motorDrive(double motorFrontLeftPower, double motorBackLeftPower, double motorFrontRightPower, double motorBackRightPower){
        motorBackLeft.setPower(motorBackLeftPower);
        motorFrontLeft.setPower(motorFrontLeftPower);
        motorBackRight.setPower(motorBackRightPower);
        motorFrontRight.setPower(motorFrontRightPower);
    }

    public void motorDriveEncoded(double motorFrontLeftPower, double motorBackLeftPower, double motorFrontRightPower, double motorBackRightPower, double distance){
        motorBackLeft.setPower(motorBackLeftPower);
        motorFrontLeft.setPower(motorFrontLeftPower);
        motorBackRight.setPower(motorBackRightPower);
        motorFrontRight.setPower(motorFrontRightPower);
    }

    public void motorDriveTime(double motorFrontLeftPower, double motorBackLeftPower, double motorFrontRightPower, double motorBackRightPower, double time){
        motorBackLeft.setPower(motorBackLeftPower);
        motorFrontLeft.setPower(motorFrontLeftPower);
        motorBackRight.setPower(motorBackRightPower);
        motorFrontRight.setPower(motorFrontRightPower);
    }

    public void motorDriveRelativeAngle(double radians, double speed){
        double spinvec = 0;
        double yvec = tan(radians)/sqrt((pow(tan(radians),2))+1)*tan(radians);
        double xvec = tan(radians)/sqrt((pow(tan(radians),2))+1);

        double y = pow(-yvec,3); // Remember, this is reversed!
        double x = pow(xvec * 1.1,3); // Counteract imperfect strafing
        double rx = pow(spinvec,3);


        //denominator is the largest motor power (absolute value) or 1
        //this ensures all the powers maintain the same ratio, but only when
        //at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
    }



    public void motorStop(){
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
    }

    public void motorDriveForward(double speed){
        motorDrive(speed, speed, speed, speed);
    }
    public void motorDriveLeft(double speed){
        motorDrive(speed, -speed, speed, -speed);
    }
    public void motorDriveRight(double speed){
        motorDrive(-speed, speed, -speed, speed);
    }
    public void motorDriveBack(double speed){
        motorDrive(-speed, -speed, -speed, -speed);
    }

    public void motorDriveForwardEncoded(double speed, double distance){
        motorDrive(speed, speed, speed, speed);
    }
    public void motorDriveLeftEncoded(double speed, double distance){
        motorDrive(speed, -speed, speed, -speed);
    }
    public void motorDriveRightEncoded(double speed, double distance){
        motorDrive(-speed, speed, -speed, speed);
    }
    public void motorDriveBackEncoded(double speed, double distance){
        motorDrive(-speed, -speed, -speed, -speed);
    }
    public void motorDriveForwardTime(double speed, double time){
        motorDrive(speed, speed, speed, speed);
        delay(time);
        motorStop();
    }
    public void motorDriveLeftTime(double speed, double time){
        motorDrive(speed, -speed, speed, -speed);
        delay(time);
        motorStop();
    }
    public void motorDriveRightTime(double speed, double time){
        motorDrive(-speed, speed, -speed, speed);
        delay(time);
        motorStop();
    }
    public void motorDriveBackTime(double speed, double time){
        motorDrive(-speed, -speed, -speed, -speed);
        delay(time);
        motorStop();
    }

    public void delay(double time){
        ElapsedTime e = new ElapsedTime();
        e.reset();
        while(e.milliseconds() < time){

        }
    }

    public void motorSpinLeft(double speed){
        motorDrive(-speed, -speed, speed, speed);
    }
    public void motorSpinRight(double speed){
        motorDrive(speed, speed, -speed, -speed);
    }

    public void motorSpinLeftEncoded(double speed){
        motorDrive(-speed, -speed, speed, speed);
    }
    public void motorSpinRightEncoded(double speed){
        motorDrive(speed, speed, -speed, -speed);
    }

    public void motorSpinLeftTime(double speed, double time){
        motorDrive(-speed, -speed, speed, speed);
        delay(time);
        motorStop();
    }
    public void motorSpinRightTime(double speed, double time){
        motorDrive(speed, speed, -speed, -speed);
        delay(time);
        motorStop();
    }


    public void spinnySpin(double speed){
        arm.setPower(speed);
    }

    public void spinnySpinForward(double speed){
        spinnySpin(speed);
    }
    public void spinnySpinBackward(double speed){
        spinnySpin(-speed);
    }

    public void spinnySpinEncoded(double speed, double target){
        double encodedSpins = 0;

        motorStop();
    }

    public void spinnySpinTime(double speed, double time){
        spinnySpin(speed);
        delay(time);
        spinnySpin(0);
    }


    public void turnRadians(double radians, double speed) {
        turnRadians(radians, speed, angles.firstAngle);
    }
    public double turnRadians(double radians, double speed, double startRadians) {
        double target = startRadians + radians;
        double minSpeed = 0.1;
        while(angles.firstAngle < target){

            if(target-angles.firstAngle>minSpeed) {
                motorSpinRight(target - angles.firstAngle);
            }else{
                motorSpinRight(minSpeed);
            }

            //telemetry.addData("Angles: ", angles.firstAngle);
            //telemetry.addData("Not Angles: ", angles.firstAngle - target);
            angles = getAngles();
            //telemetry.update();
        }


        motorStop();
        return target-startRadians;

    }



    public Orientation getAngles() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }
}
