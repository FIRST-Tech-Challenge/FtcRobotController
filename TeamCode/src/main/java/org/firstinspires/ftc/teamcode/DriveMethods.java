package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Variables.DISTANCE_PER_CLICK;
import static org.firstinspires.ftc.teamcode.Variables.motorBackLeft;
import static org.firstinspires.ftc.teamcode.Variables.motorBackRight;
import static org.firstinspires.ftc.teamcode.Variables.motorFrontLeft;
import static org.firstinspires.ftc.teamcode.Variables.motorFrontRight;

public class DriveMethods extends LinearOpMode {
 boolean calibrated = false;
    // Ignore this method (it is to satisfy the parent class LinearOpmode)
    @Override
    public void runOpMode() throws InterruptedException {

    }

    public enum Direction {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT,


        FORWARD_LEFT,
        BACKWARD_LEFT,
        FORWARD_RIGHT,
        BACKWARD_RIGHT,
        TURN_LEFT,
        TURN_RIGHT,
    }

    public void driveDirection(double power, Direction direction) {

        switch(direction) {
            case FORWARD:
                motorFrontLeft.setPower(power);
                motorBackLeft.setPower(power);
                motorFrontRight.setPower(power);
                motorBackRight.setPower(power);
                break;

            case BACKWARD:
                motorFrontLeft.setPower(-power);
                motorBackLeft.setPower(-power);
                motorFrontRight.setPower(-power);
                motorBackRight.setPower(-power);
                break;

            case LEFT:
                motorFrontLeft.setPower(-power);
                motorBackLeft.setPower(power);
                motorFrontRight.setPower(power);
                motorBackRight.setPower(-power);
                break;

            case RIGHT:
                motorFrontLeft.setPower(power);
                motorBackLeft.setPower(-power);
                motorFrontRight.setPower(-power);
                motorBackRight.setPower(power);
                break;

            case FORWARD_LEFT:
                motorFrontLeft.setPower(0.0);
                motorBackLeft.setPower(power);
                motorFrontRight.setPower(power);
                motorBackRight.setPower(0.0);
                break;

            case BACKWARD_LEFT:
                motorFrontLeft.setPower(-power);
                motorBackLeft.setPower(0.0);
                motorFrontRight.setPower(0.0);
                motorBackRight.setPower(-power);
                break;

            case FORWARD_RIGHT:
                motorFrontLeft.setPower(power);
                motorBackLeft.setPower(0.0);
                motorFrontRight.setPower(0.0);
                motorBackRight.setPower(power);
                break;

            case BACKWARD_RIGHT:
                motorFrontLeft.setPower(0.0);
                motorBackLeft.setPower(-power);
                motorFrontRight.setPower(-power);
                motorBackRight.setPower(0.0);
                break;

            case TURN_LEFT:
                motorFrontLeft.setPower(-power);
                motorBackLeft.setPower(-power);
                motorFrontRight.setPower(power);
                motorBackRight.setPower(power);
                break;

            case TURN_RIGHT:
                motorFrontLeft.setPower(power);
                motorBackLeft.setPower(power);
                motorFrontRight.setPower(-power);
                motorBackRight.setPower(-power);
                break;
        }
    }
    public void driveForDistance (double distance, double power, Direction direction) {

        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double distanceTraveled = 0;
        int TARGET_POSITION = (int)((distance/DISTANCE_PER_CLICK));
        double ScaleFactorForStrafing = 0.92;

        if(direction == Direction.LEFT || direction == Direction.RIGHT){
            motorFrontRight.setTargetPosition((int)(TARGET_POSITION/ScaleFactorForStrafing));
            motorFrontLeft.setTargetPosition((int)(TARGET_POSITION/ScaleFactorForStrafing));
            motorBackRight.setTargetPosition((int)(TARGET_POSITION/ScaleFactorForStrafing));
            motorBackLeft.setTargetPosition((int)(TARGET_POSITION/ScaleFactorForStrafing));

        } else {

            motorFrontRight.setTargetPosition((TARGET_POSITION));
            motorFrontLeft.setTargetPosition((TARGET_POSITION));
            motorBackRight.setTargetPosition((TARGET_POSITION));
            motorBackLeft.setTargetPosition((TARGET_POSITION));

        }

//        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        if (PID) {
//            while (distanceTraveled < TARGET_POSITION) {
//                distanceTraveled = (abs(motorFrontLeft.getCurrentPosition()) + abs(motorFrontRight.getCurrentPosition()) + abs(motorBackLeft.getCurrentPosition()) + abs(motorBackRight.getCurrentPosition())) / 4;
//                telemetry.addLine("Driving using PID control.");
//                telemetry.addLine("TARGET_POSITION: " + TARGET_POSITION);
//                telemetry.addLine("distanceTraveled: " + distanceTraveled);
//                PofPID(power, distance, direction, ScaleFactorForStrafing);
//
//            }
//
//        } else {
            driveDirection(power,direction);
            while (distanceTraveled < TARGET_POSITION) {
                distanceTraveled = (abs(motorFrontLeft.getCurrentPosition()) + abs(motorFrontRight.getCurrentPosition()) + abs(motorBackLeft.getCurrentPosition()) + abs(motorBackRight.getCurrentPosition())) / 4;
                telemetry.addLine("Driving at manual speed " + power + " for " + distance + " meters " + direction);
                telemetry.addLine("Distance Traveled (In clicks): " + distanceTraveled);
                telemetry.addLine("Distance Inputted (In clicks): " + TARGET_POSITION);
                telemetry.addLine("Distance Per Click: " + DISTANCE_PER_CLICK);
                telemetry.update();
//            }
        }

        StopMotors();

    }
    public void PofPID(double power, double distance, Direction direction, double scalefactorstrafing) {
        double distTraveled = ((motorBackLeft.getCurrentPosition() * DISTANCE_PER_CLICK) + (motorBackRight.getCurrentPosition() * DISTANCE_PER_CLICK) + (motorFrontLeft.getCurrentPosition() * DISTANCE_PER_CLICK) + (motorFrontRight.getCurrentPosition() * DISTANCE_PER_CLICK)) / 4;
        double distTraveledStrafing = scalefactorstrafing*(((motorBackLeft.getCurrentPosition() * DISTANCE_PER_CLICK) + (motorBackRight.getCurrentPosition() * DISTANCE_PER_CLICK) + (motorFrontLeft.getCurrentPosition() * DISTANCE_PER_CLICK) + (motorFrontRight.getCurrentPosition() * DISTANCE_PER_CLICK)) / 4);
        if(direction == direction.LEFT || direction == direction.RIGHT){
            if ((distance - distTraveledStrafing) > 0.5) {
                driveDirection(power, direction);
            } else if ((distance - distTraveled) < 0.1) {
                power = 0.1;
                driveDirection(power, direction);
            }else {

                power = 2 * (distance - distTraveledStrafing) * power;
                driveDirection(power, direction);

            }
            telemetry.addLine("Moving at a speed: " + power);
            telemetry.addLine("Your robot has currently traveled " + distTraveledStrafing + " of " + distance + " meters." );
            telemetry.update();
        }else {
            if ((distance - distTraveled) > 0.5) {
                driveDirection(power, direction);
            } else if ((distance - distTraveled) < 0.1) {
                power = 0.1;
                driveDirection(power, direction);
            } else{

                power = 2 * (distance - distTraveled) * power;
                driveDirection(power, direction);

            }
            telemetry.addLine("Moving at a speed: " + power);
            telemetry.addLine("Your robot has currently traveled " + distTraveled + " of " + distance + " meters." );
            telemetry.update();
        }

    }
    public void StopMotors(){
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }
    public void calibrateIMU(){
        BNO055IMU imu;
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        calibrated = true;
    }

    public double getCurrentZAngle () {
        if (calibrated == false){
            calibrateIMU();
        }else{

        }
        BNO055IMU imu;
        imu = hardwareMap.get(BNO055IMU.class, "imu");


        Orientation CurrentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentZ = CurrentAngle.firstAngle;
        return currentZ;

    }

    public void rotateToPosition(double power, double position){

        double currentZ = getCurrentZAngle();
        double difference = currentZ - position;
        if(difference < 0){
            position = position - 5;
            while(currentZ < position){
                driveDirection(power, Direction.TURN_LEFT);
                currentZ = getCurrentZAngle();
                difference = position - currentZ;
                telemetry.addLine("Current Z:" + currentZ);
                telemetry.addLine("Rotating to: " + position);
                telemetry.update();
                if (difference < 15) {
                    while (currentZ < position) {
                        driveDirection(0.1, Direction.TURN_LEFT);
                        currentZ = getCurrentZAngle();
                        difference = position - currentZ;
                        telemetry.addLine("Current Z:" + currentZ);
                        telemetry.addLine("Rotating to: " + position);
                        telemetry.update();
                    }
                }

            }
        }else if (difference > 0){
            position = position + 5;
            while(currentZ > position){
                driveDirection(power, Direction.TURN_RIGHT);
                currentZ = getCurrentZAngle();
                difference = position - currentZ;
                telemetry.addLine("Current Z:" + currentZ);
                telemetry.addLine("Rotating to: " + position);
                telemetry.update();
                if(difference > -15){
                    while(currentZ > position) {
                        driveDirection(0.1, Direction.TURN_RIGHT);
                        currentZ = getCurrentZAngle();
                        difference = position - currentZ;
                        telemetry.addLine("Current Z:" + currentZ);
                        telemetry.addLine("Rotating to: " + position);
                        telemetry.update();
                    }
                }
            }
        }
        StopMotors();
    }

}
