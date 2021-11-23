package org.wheelerschool.robotics.comp.chassis;

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

    private void motorDriveEncoded(double motorFrontLeftPower, double motorBackLeftPower, double motorFrontRightPower, double motorBackRightPower, double distance){
        motorBackLeft.setPower(motorBackLeftPower);
        motorFrontLeft.setPower(motorFrontLeftPower);
        motorBackRight.setPower(motorBackRightPower);
        motorFrontRight.setPower(motorFrontRightPower);
    }

    private void motorDriveTime(double motorFrontLeftPower, double motorBackLeftPower, double motorFrontRightPower, double motorBackRightPower, double time){
        motorBackLeft.setPower(motorBackLeftPower);
        motorFrontLeft.setPower(motorFrontLeftPower);
        motorBackRight.setPower(motorBackRightPower);
        motorFrontRight.setPower(motorFrontRightPower);
    }

    private void motorDriveRelativeAngle(double radians, double speed){
        double spinvec = 0;
        double yvec = tan(radians)/sqrt((tan(radians)^2)+1)*tan(radians)
        double xvec = tan(radians)/sqrt((tan(radians)^2)+1)

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
        motorDrive(speed, speed, speed, speed);
    }
    private void motorDriveLeftEncoded(double speed, double distance){
        motorDrive(speed, -speed, speed, -speed);
    }
    private void motorDriveRightEncoded(double speed, double distance){
        motorDrive(-speed, speed, -speed, speed);
    }
    private void motorDriveBackEncoded(double speed, double distance){
        motorDrive(-speed, -speed, -speed, -speed);
    }
    private void motorDriveForwardTime(double speed, double time){
        motorDrive(speed, speed, speed, speed);
    }
    private void motorDriveLeftTime(double speed, double time){
        motorDrive(speed, -speed, speed, -speed);
    }
    private void motorDriveRightTime(double speed, double time){
        motorDrive(-speed, speed, -speed, speed);
    }
    private void motorDriveBackTime(double speed, double time){
        motorDrive(-speed, -speed, -speed, -speed);
    }


    private void motorSpinLeft(double speed){
        motorDrive(-speed, -speed, speed, speed);
    }
    private void motorSpinRight(double speed){
        motorDrive(speed, speed, -speed, -speed);
    }

    private void motorSpinLeftEncoded(double speed){
        motorDrive(-speed, -speed, speed, speed);
    }
    private void motorSpinRightEncoded(double speed){
        motorDrive(speed, speed, -speed, -speed);
    }

    private void motorSpinLeftTime(double speed, double time){
        motorDrive(-speed, -speed, speed, speed);
        delay(time);
        motorStop();
    }
    private void motorSpinRightTime(double speed, double time){
        motorDrive(speed, speed, -speed, -speed);
        delay(time);
        motorStop();
    }


    private void spinnySpin(double speed){
        arm.setPower(speed);
    }

    private void spinnySpinForward(double speed){
        spinnySpin(speed);
    }
    private void spinnySpinBackward(double speed){
        spinnySpin(-speed);
    }

    private void spinnySpinEncoded(double speed, double target){
        double encodedSpins = 0;
        while (opModeIsActive() && encodedSpins < rotation){
            encodedSpins = 0;
        }
        motorStop();
    }

    private void spinnySpinEncoded(double speed, double spins){
        double encodedSpins = 0;
        while (opModeIsActive() && encodedSpins < spins){
            spins--;
        }
        motorStop();
    }


    private void turnRadians(double radians, double speed) {
        turnRadians(radians, speed, angles.firstAngle);
    }
    private double turnRadians(double radians, double speed, double startRadians) {
        double target = startRadians + radians;
        double minSpeed = 0.1;
        while(angles.firstAngle < target && opModeIsActive()){

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
