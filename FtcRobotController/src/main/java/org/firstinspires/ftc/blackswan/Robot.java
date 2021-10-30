package org.firstinspires.ftc.blackswan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.abs;

public class Robot {
    Telemetry telemetry;
    DcMotor frontLeft, backLeft, frontRight,backRight;
    CRServo carousel;
    LinearOpMode opMode;

    final int TICKS_PER_ROTATION= 537;

    public void stopMotors(){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode){
        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        backRight = hardwareMap.get(DcMotor.class,"backRight");

        carousel = hardwareMap.get(CRServo.class, "carousel");

        this.opMode= opMode;
    }

    public void forward(double distance, double speed){

         double distanceInTicks= distance*TICKS_PER_ROTATION;

        int frontLeftPosition= frontLeft.getCurrentPosition();

        int frontLeftTarget= frontLeftPosition+ (int)distanceInTicks;

        int frontRightPosition= frontRight.getCurrentPosition();

        int backLeftPosition= backLeft.getCurrentPosition();

        int backLeftTarget= backLeftPosition+ (int)distanceInTicks;

        int backRightPosition= backRight.getCurrentPosition();

        int backRightTarget= backRightPosition+ (int)distanceInTicks;

        int frontRightTarget= frontRightPosition+ (int)distanceInTicks;
        frontLeft.setTargetPosition((int)frontLeftTarget);
        frontRight.setTargetPosition((int)frontRightTarget);
        backLeft.setTargetPosition((int)backLeftTarget);
        backRight.setTargetPosition((int)backRightTarget);

        motorSetMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(abs(speed));
        frontRight.setPower(abs(speed));
        backLeft.setPower(abs(speed));
        backRight.setPower(abs(speed));

        while (this.opMode.opModeIsActive() &&
                (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

            // Display it for the driver.

        }
        stopMotors();

        motorSetMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }
    public void backward(double distance, double speed){
        forward(-distance, speed);
    }
    public void left(double distance, double speed){

    }
    public void right(double distance, double speed){

    }
    public void turnLeft(double distance, double speed){

    }
    public void turnRight(double distance, double speed){

    }

    protected void motorSetMode(DcMotor.RunMode runMode){
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
    }
}
