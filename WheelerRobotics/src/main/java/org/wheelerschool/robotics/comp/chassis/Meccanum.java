package org.wheelerschool.robotics.comp.chassis;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Math.abs;
import static java.lang.Math.min;
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

import java.util.ArrayList;

public class Meccanum {

    private ElapsedTime runtime = new ElapsedTime();

    private Servo servo0;
    private DcMotor arm;

    public final double NORMAL_SPEED = 0.5; // preference and feel for best
    public final double SERVO_FULLY_CLOSED = 0; // need arm+hub to test this
    public final double SERVO_FULLY_OPENED = 0; // need arm+hub to test this
    public final double HALF_SERVO_ANGLE = 0;
    public final double ARM_MAX_SPEED = 0; // preference? or maybe to be precise
    public final double HIGH_SPINNER_POWER = 1; // probably max, may need to adjust later
    public final double OPTIMAL_SPINNER_POWER = 0; // need spinner+hub to test this
    public final double MOTOR_STOP = 0; // its just 0 cuz full stop


    private DcMotor spinner;

    private BNO055IMU imu;

    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;

    private Orientation angles;

    private volatile HardwareMap hw; // no idea if volatile means anything (impactful) in this context, but it makes me seem like I know what im doing

    public void init(HardwareMap hardwareMap){
        // internal IMU setup

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

        hw = hardwareMap;

        runtime.reset();
    }


    public void motorDrive(double motorFrontLeftPower, double motorBackLeftPower, double motorFrontRightPower, double motorBackRightPower){
        motorBackLeft.setPower(motorBackLeftPower);
        motorFrontLeft.setPower(motorFrontLeftPower);
        motorBackRight.setPower(motorBackRightPower);
        motorFrontRight.setPower(motorFrontRightPower);
    }

    public void motorDriveEncoded(double motorFrontLeftPower, double motorBackLeftPower, double motorFrontRightPower, double motorBackRightPower, double ticks){
        motorDriveEncoded(motorFrontLeftPower, motorBackLeftPower, motorFrontRightPower, motorBackRightPower, ticks, motorBackLeft.getCurrentPosition(), motorFrontLeft.getCurrentPosition(), motorBackRight.getCurrentPosition(), motorFrontRight.getCurrentPosition());
    }
    private void motorDriveEncoded(double motorFrontLeftPower, double motorBackLeftPower, double motorFrontRightPower, double motorBackRightPower, double ticks, int blp, int flp, int brp, int frp){ // private I think bcuz only ever accessed inside the class
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
        final int blp = motorBackLeft.getCurrentPosition(); // idk if this will stay a static value or if it will change with the motor pos, hmm...
        final int brp = motorBackRight.getCurrentPosition();
        final int frp = motorFrontRight.getCurrentPosition();
        final int flp = motorFrontLeft.getCurrentPosition();
        */

        int blip = blp; // back left initial position
        int flip = flp;
        int brip = brp;
        int frip = frp;

        while(abs(motorBackLeft.getCurrentPosition() - blip) < ticks) { // hopefully checks that it is within the positive or negative threshold of target ticks
            motorDrive(motorFrontLeftPower, motorBackLeftPower, motorFrontRightPower, motorBackRightPower);
        }
        motorStop();
    }

    public void motorDriveTime(double motorFrontLeftPower, double motorBackLeftPower, double motorFrontRightPower, double motorBackRightPower, double time){
        motorBackLeft.setPower(motorBackLeftPower);
        motorFrontLeft.setPower(motorFrontLeftPower);
        motorBackRight.setPower(motorBackRightPower);
        motorFrontRight.setPower(motorFrontRightPower);
        delay(time);
        motorStop();
    }

    public void motorDriveRelativeAngle(double radians, double speed){ //test on monday 11/29/2021
        //NOTE
        // im not sure how to acurately do this using encoders, because some wheels are going to spin at different powers (I think)
        // this will cause the ticks to be difficult to calculate, and I dont really want to deal with that rn

        double spinvec = 0; // not spinning
        double yvec = min(speed, 1.0) * tan(radians)/sqrt(pow(tan(radians),2)+1)*tan(radians); // ima be honest, i did this math for a js project 6 months ago and am just hopin it actually works in this context
        double xvec = min(speed, 1.0) * tan(radians)/sqrt(pow(tan(radians),2))+1;

        double y = pow(-yvec,3); // Remember, this is reversed!
        double x = pow(xvec * 1.1,3); // Counteract imperfect strafing
        double rx = pow(spinvec,3);


        //denominator is the largest motor power (absolute value) or 1
        //this ensures all the powers maintain the same ratio, but only when
        //at least one is out of the range [-1, 1]
        double denominator = Math.max(abs(y) + abs(x) + abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motorDrive(frontLeftPower,backLeftPower, frontRightPower, backRightPower);

    }

    public void motorDriveRelativeAngleTime(double radians, double speed, double time){
        motorDriveRelativeAngle(radians, speed);
        delay(time);
        motorStop();
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
        motorDriveEncoded(speed, speed, speed, speed, distance);
    }
    public void motorDriveLeftEncoded(double speed, double distance){
        motorDriveEncoded(speed, -speed, speed, -speed, distance);
    }
    public void motorDriveRightEncoded(double speed, double distance){
        motorDriveEncoded(-speed, speed, -speed, speed, distance);
    }
    public void motorDriveBackEncoded(double speed, double distance){
        motorDriveEncoded(-speed, -speed, -speed, -speed, distance);
    }
    public void motorDriveForwardTime(double speed, double time){
        motorDriveTime(speed, speed, speed, speed, time);
    }
    public void motorDriveLeftTime(double speed, double time){
        motorDriveTime(speed, -speed, speed, -speed, time);
    }
    public void motorDriveRightTime(double speed, double time){
        motorDriveTime(-speed, speed, -speed, speed, time);
    }
    public void motorDriveBackTime(double speed, double time){
        motorDriveTime(-speed, -speed, -speed, -speed, time);
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

    public void motorSpinLeftEncoded(double speed, double distance){
        motorDriveEncoded(-speed, -speed, speed, speed, distance);
    }
    public void motorSpinRightEncoded(double speed, double distance){
        motorDriveEncoded(speed, speed, -speed, -speed, distance);
    }

    public void motorSpinLeftTime(double speed, double time){
        motorDriveTime(-speed, -speed, speed, speed, time);
    }
    public void motorSpinRightTime(double speed, double time){
        motorDriveTime(speed, speed, -speed, -speed, time);
    }


    public void spinnySpin(double speed){
        arm.setPower(speed);
    }

    public void spinnySpinEncoded(double speed, double target){
        spinnySpinEncoded(speed, target, spinner.getCurrentPosition());
    }
    public void spinnySpinEncoded(double speed, double target, int start){

        while (abs(spinner.getCurrentPosition()-start) < target){
            spinnySpin(speed);
        }
        spinnyStop();
    }

    public void spinnyStop() {
        spinner.setPower(0);
    }

    public void spinnySpinTime(double speed, double time){
        spinnySpin(speed);
        delay(time);
        spinnyStop();
    }


    public void turnRadians(double radians, double speed) {
        turnRadians(radians, speed, angles.firstAngle);
    }

    private double turnRadians(double radians, double speed, double startRadians) { // private I think bcuz only ever accessed inside the class
        double target = startRadians + radians;
        double minSpeed = 0.1;
        while(angles.firstAngle < target){

            if(target-angles.firstAngle>minSpeed) {
                motorSpinRight(target - angles.firstAngle);
            }else{
                motorSpinRight(minSpeed);
            }

            telemetry.addData("Angles: ", angles.firstAngle);
            telemetry.addData("Not Angles: ", angles.firstAngle - target);
            angles = getAngles();
            telemetry.update();
        }


        motorStop();
        return target-startRadians;


    }

    public void setServo(double angle){
        servo0.setPosition(angle);
    }

    public void openServoHalf(){
        servo0.setPosition(HALF_SERVO_ANGLE);
    }

    public void openServoFull(){
        servo0.setPosition(SERVO_FULLY_OPENED);
    }

    public void closeServoFull(){
        servo0.setPosition(SERVO_FULLY_CLOSED);
    }

    public Orientation getAngles() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }
}
