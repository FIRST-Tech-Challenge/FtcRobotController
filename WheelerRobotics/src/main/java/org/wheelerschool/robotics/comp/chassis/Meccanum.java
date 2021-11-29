package org.wheelerschool.robotics.comp.chassis;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Math.abs;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    private final double SERVO_FULLY_CLOSED = 0;
    private final double SERVO_FULLY_OPENED = 0;
    private final double ARM_MAX_SPEED = 0;
    private final double HIGH_SPINNER_POWER = 0;
    private final double OPTIMAL_SPINNER_POWER = 0;
    private final double MOTOR_STOP = 0;

    private DcMotor spinner;

    private BNO055IMU imu;

    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;

    private Orientation angles;

    public void init(HardwareMap hardwareMap){

    }


    private void motorDrive(double motorFrontLeftPower, double motorBackLeftPower, double motorFrontRightPower, double motorBackRightPower){
        motorBackLeft.setPower(motorBackLeftPower);
        motorFrontLeft.setPower(motorFrontLeftPower);
        motorBackRight.setPower(motorBackRightPower);
        motorFrontRight.setPower(motorFrontRightPower);
    }

    private void motorDriveEncoded(double motorFrontLeftPower, double motorBackLeftPower, double motorFrontRightPower, double motorBackRightPower, double ticks){
        motorDriveEncoded(motorFrontLeftPower, motorBackLeftPower, motorFrontRightPower, motorBackRightPower, ticks, motorBackLeft.getCurrentPosition(), motorFrontLeft.getCurrentPosition(), motorBackRight.getCurrentPosition(), motorFrontRight.getCurrentPosition());
    }
    private void motorDriveEncoded(double motorFrontLeftPower, double motorBackLeftPower, double motorFrontRightPower, double motorBackRightPower, double ticks, int blp, int flp, int brp, int frp){
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

    private void motorDriveTime(double motorFrontLeftPower, double motorBackLeftPower, double motorFrontRightPower, double motorBackRightPower, double time){
        motorBackLeft.setPower(motorBackLeftPower);
        motorFrontLeft.setPower(motorFrontLeftPower);
        motorBackRight.setPower(motorBackRightPower);
        motorFrontRight.setPower(motorFrontRightPower);
        delay(time);
        motorStop();
    }

    private void motorDriveRelativeAngle(double radians, double speed){ //test on monday 11/29/2021
        //NOTE
        // im not sure how to acurately do this using encoders, because some wheels are going to spin at different powers (I think)
        // this will cause the ticks to be difficult to calculate, and I dont really want to deal with that rn

        double spinvec = 0;
        double yvec = tan(radians)/sqrt(pow(tan(radians),2)+1)*tan(radians);
        double xvec = tan(radians)/sqrt(pow(tan(radians),2))+1;

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

    private void motorDriveRelativeAngleTime(double radians, double speed, double time){
        motorDriveRelativeAngle(radians, speed);
        delay(time);
        motorStop();
    }


    private void motorStop(){
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
    }

    private void motorDriveForward(double speed){
        motorDrive(speed, speed, speed, speed);
    }
    private void motorDriveLeft(double speed){
        motorDrive(speed, -speed, speed, -speed);
    }
    private void motorDriveRight(double speed){
        motorDrive(-speed, speed, -speed, speed);
    }
    private void motorDriveBack(double speed){
        motorDrive(-speed, -speed, -speed, -speed);
    }

    private void motorDriveForwardEncoded(double speed, double distance){
        motorDriveEncoded(speed, speed, speed, speed, distance);
    }
    private void motorDriveLeftEncoded(double speed, double distance){
        motorDriveEncoded(speed, -speed, speed, -speed, distance);
    }
    private void motorDriveRightEncoded(double speed, double distance){
        motorDriveEncoded(-speed, speed, -speed, speed, distance);
    }
    private void motorDriveBackEncoded(double speed, double distance){
        motorDriveEncoded(-speed, -speed, -speed, -speed, distance);
    }
    private void motorDriveForwardTime(double speed, double time){
        motorDriveTime(speed, speed, speed, speed, time);
    }
    private void motorDriveLeftTime(double speed, double time){
        motorDriveTime(speed, -speed, speed, -speed, time);
    }
    private void motorDriveRightTime(double speed, double time){
        motorDriveTime(-speed, speed, -speed, speed, time);
    }
    private void motorDriveBackTime(double speed, double time){
        motorDriveTime(-speed, -speed, -speed, -speed, time);
    }

    private void delay(double time){
        ElapsedTime e = new ElapsedTime();
        e.reset();
        while(e.milliseconds() < time){

        }
    }

    private void motorSpinLeft(double speed){
        motorDrive(-speed, -speed, speed, speed);
    }
    private void motorSpinRight(double speed){
        motorDrive(speed, speed, -speed, -speed);
    }

    private void motorSpinLeftEncoded(double speed, double distance){
        motorDriveEncoded(-speed, -speed, speed, speed, distance);
    }
    private void motorSpinRightEncoded(double speed, double distance){
        motorDriveEncoded(speed, speed, -speed, -speed, distance);
    }

    private void motorSpinLeftTime(double speed, double time){
        motorDriveTime(-speed, -speed, speed, speed, time);
    }
    private void motorSpinRightTime(double speed, double time){
        motorDriveTime(speed, speed, -speed, -speed, time);
    }


    private void spinnySpin(double speed){
        arm.setPower(speed);
    }

    private void spinnySpinEncoded(double speed, double target){
        spinnySpinEncoded(speed, target, spinner.getCurrentPosition());
    }
    private void spinnySpinEncoded(double speed, double target, int start){

        while (abs(spinner.getCurrentPosition()-start) < target){
            spinnySpin(speed);
        }
        spinnyStop();
    }

    private void spinnyStop() {
        spinner.setPower(0);
    }

    private void spinnySpinTime(double speed, double time){
        spinnySpin(speed);
        delay(time);
        spinnyStop();
    }


    private void turnRadians(double radians, double speed) {
        turnRadians(radians, speed, angles.firstAngle);
    }

    private double turnRadians(double radians, double speed, double startRadians) { // idrk about this rn, but it seems useful in a scenario without my drive radians function
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



    private Orientation getAngles() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }
}
