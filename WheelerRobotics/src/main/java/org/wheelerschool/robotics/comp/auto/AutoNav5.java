package org.wheelerschool.robotics.comp.auto;

import static com.sun.tools.doclint.Entity.and;
import static java.lang.Math.floor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutoNav5 extends LinearOpMode {

    BNO055IMU imu;
    DcMotor motorFrontLeft = null;
    DcMotor motorBackLeft = null;
    DcMotor motorFrontRight = null;
    DcMotor motorBackRight = null;
    private ElapsedTime runtime = new ElapsedTime();
    private double seconds = runtime.seconds();





    @Override
    public void runOpMode() throws InterruptedException {

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();
        while(opModeIsActive()){
            telemetry.addData("Elapsed Time  ", floor(runtime.seconds() / 60) + ":" + runtime.seconds() % 60 + ":" + runtime.milliseconds() % 1000 );

            if(runtime.seconds()>1){
                motorStop();
            }else{
                motorDriveForward(1);
            }
            // 685mm  0.5power 1sec
            telemetry.addData("GYRO", imu.getAngularOrientation());

            telemetry.update();


        }
    }

    private void motorDrive(double motorFrontLeftPower, double motorBackLeftPower, double motorFrontRightPower, double motorBackRightPower){
        motorBackLeft.setPower(motorBackLeftPower);
        motorFrontLeft.setPower(motorFrontLeftPower);
        motorBackRight.setPower(motorBackRightPower);
        motorFrontRight.setPower(motorFrontRightPower);
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
    private void motorSpinLeft(double speed){
        motorDrive(-speed, -speed, speed, speed);
    }
    private void motorSpinRight(double speed){
        motorDrive(speed, speed, -speed, -speed);
    }
    private void turnRadians(double speed) {

    }

}
