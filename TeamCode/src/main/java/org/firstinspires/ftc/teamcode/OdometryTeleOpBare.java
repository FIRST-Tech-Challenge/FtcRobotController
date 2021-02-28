package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.OdometryGlobalCoordinatePosition;


/**
 * Main Teleop
 *
 * 3 October 2020
 */

@TeleOp(name = "OdometryTeleopBare")
public class OdometryTeleOpBare extends LinearOpMode{
    private DcMotor motorFrontRight, motorFrontLeft, motorBackLeft, motorBackRight;

    private DcMotor outtakeRight, wobbleArm;

    private BNO055IMU imu;

    private IMURobot robot;

    //Figures for Odometry
    final double COUNTS_PER_INCH = 307.699557;



    //Odometry encoder wheels
    DcMotor verticalRight, verticalLeft, horizontal;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");

        wobbleArm = hardwareMap.dcMotor.get("wobbleArm");

        outtakeRight = hardwareMap.dcMotor.get("outtakeRight");
        outtakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Encoders
        /*
        verticalLeft = hardwareMap.dcMotor.get("FL");
        verticalRight = hardwareMap.dcMotor.get("FR");
        horizontal = hardwareMap.dcMotor.get("BL");
         */
        horizontal = hardwareMap.dcMotor.get("outtakeRight");
        verticalLeft = hardwareMap.dcMotor.get("wobbleArm");
        verticalRight = hardwareMap.dcMotor.get("intake");


        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();

        while(opModeIsActive()){

            if (gamepad1.left_bumper){
                odometryDriveToPosCorrected(10,10,0);
            }

            if (gamepad1.x){
                odometryTurnDifference();
            }

            if (gamepad1.b){
                odometryTurnInequality();
            }

            if (gamepad1.a){
                setOdometryAngle(0);
            }

            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - (Math.PI / 4);
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;

            double powerOne = r * Math.sin(angle);
            double powerTwo = r * Math.cos(angle);

            motorFrontLeft.setPower(powerOne - (rotation));
            motorFrontRight.setPower(powerTwo + (rotation));
            motorBackLeft.setPower(powerTwo - (rotation));
            motorBackRight.setPower(powerOne + (rotation));

            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("Orientation Raw (Degrees)", globalPositionUpdate.returnOrientationRaw());


            telemetry.addData("Relative Angle Difference from 0: ", getOdometryAngleDifference(0));
            telemetry.addData("Relative Angle Difference from 90: ", getOdometryAngleDifference(90));
            telemetry.addData("Raw Angle Difference from 0: ", getAngleRaw(0));
            telemetry.addData("Raw Angle Difference from 90: ", getAngleRaw(90));


            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());


            telemetry.addData("Thread Active", positionThread.isAlive());

            telemetry.update();
            idle();
        }
        globalPositionUpdate.stop();
        telemetry.addData("Thread Active", positionThread.isAlive());
        telemetry.update();

    }

    public void odometryDriveToPos (double xPos, double yPos, double direction) {
        //setOdometryAngle(0);
        double distanceX = xPos - (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);//0
        double distanceY = yPos - (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);//0

        double angle = (Math.atan2(distanceY,distanceX)-(Math.PI/4));
        double distance = Math.hypot(distanceX,distanceY);//0

        double powerOne = 1 * Math.sin(angle);
        double powerTwo = 1 * Math.cos(angle);

        while (distance > 1){
            if (gamepad1.y){
                break;
            }

            distanceX = xPos - (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            distanceY = yPos - (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            distance = Math.hypot(distanceX,distanceY);

            angle = (Math.atan2(distanceY,distanceX)-(Math.PI/4));
            if (distance >= 10){
                powerOne = 1 * Math.sin(angle);
                powerTwo = 1 * Math.cos(angle);
            }else if (distance < 10 && distance > 5){
                powerOne = 0.4 * Math.sin(angle);
                powerTwo = 0.4 * Math.cos(angle);
            }else if (distance <= 5){
                powerOne = 0.3 * Math.sin(angle);
                powerTwo = 0.3 * Math.cos(angle);
            }else{
                break;
            }

            motorFrontLeft.setPower(powerOne);
            motorFrontRight.setPower(powerTwo);
            motorBackLeft.setPower(powerTwo);
            motorBackRight.setPower(powerOne);
        }
        completeStop();
        //setOdometryAngle(direction);
    }

    public void odometryTurnInequality() throws InterruptedException{
        telemetry.addData("Current Angle", globalPositionUpdate.returnOrientation());

        while (globalPositionUpdate.returnOrientation() > 20){
            turnClockwise(0.2);
            telemetry.addData("Current Angle", globalPositionUpdate.returnOrientation());
            telemetry.update();
        }
        telemetry.addData("Current Angle", globalPositionUpdate.returnOrientation());
        telemetry.update();
        sleep(5000);
        completeStop();
    }

    public void odometryTurnDifference(){
        double relativeAngleDifference = getOdometryAngleDifference(20);
        telemetry.addData("Current Angle", globalPositionUpdate.returnOrientation());

        while (relativeAngleDifference > 1){
            relativeAngleDifference = getOdometryAngleDifference(20);
            turnClockwise(0.2);
            telemetry.addData("Current Angle", globalPositionUpdate.returnOrientation());
            telemetry.update();
        }
        telemetry.addData("Current Angle", globalPositionUpdate.returnOrientation());
        telemetry.update();
        sleep(5000);
        completeStop();
    }

    public double getOdometryAngleDifference(double desiredAngle){
        double angleDifference = getAngleRaw(desiredAngle);

        if (angleDifference > 180){
            angleDifference = 360 - angleDifference;
        }

        return angleDifference;
    }

    public double getOdometryAngleDifferenceNegative(double desiredAngle){
        double angleDifference = getAngleRaw(desiredAngle);

        if (angleDifference > 180){
            angleDifference = angleDifference - 360;
        }

        return angleDifference;
    }

    public double getAngleRaw (double desiredAngle){
        return Math.abs(desiredAngle - globalPositionUpdate.returnOrientation());
    }

    public void turnClockwise(double power){
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
        motorFrontRight.setPower(-power);
        motorBackRight.setPower(-power);
    }
    public void turnCounterClockwise(double power){
        motorFrontLeft.setPower(-power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);
    }

    public void odometryDriveToPosCorrected (double xPos, double yPos, double direction) {
        //setOdometryAngle(0);
        double distanceX = xPos - (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
        double distanceY = yPos - (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);

        double angle = (Math.atan2(distanceY,distanceX)-(Math.PI/4));
        double distance = Math.hypot(distanceX,distanceY);

        double angleDifference = getOdometryAngleDifferenceNegative(direction);
        double correction = angleDifference*0.01;

        double powerOne = 1 * Math.sin(angle);
        double powerTwo = 1 * Math.cos(angle);

        while (distance > 1){
            if (gamepad1.y){
                break;
            }
            angleDifference = getOdometryAngleDifferenceNegative(direction);
            correction = angleDifference*0.01;

            distanceX = xPos - (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            distanceY = yPos - (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            distance = Math.hypot(distanceX,distanceY);

            angle = (Math.atan2(distanceY,distanceX)-(Math.PI/4));
            if (distance >= 10){
                powerOne = 0.7 * Math.sin(angle);
                powerTwo = 0.7 * Math.cos(angle);
            }else if (distance < 10 && distance > 5){
                powerOne = 0.4 * Math.sin(angle);
                powerTwo = 0.4 * Math.cos(angle);
            }else if (distance <= 5){
                powerOne = 0.3 * Math.sin(angle);
                powerTwo = 0.3 * Math.cos(angle);
            }else{
                break;
            }

            motorFrontLeft.setPower(powerOne+correction);
            motorFrontRight.setPower(powerTwo-correction);
            motorBackLeft.setPower(powerTwo+correction);
            motorBackRight.setPower(powerOne-correction);
        }
        completeStop();
        //setOdometryAngle(direction);
    }

    public void completeStop(){
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    public void setOdometryAngle(double desiredAngle) {

        double angleDifference = getOdometryAngleDifferenceNegative(desiredAngle);

        while (Math.abs(angleDifference) > 2){
            if (gamepad1.y){
                break;
            }

            angleDifference = getOdometryAngleDifferenceNegative(desiredAngle);

            if (angleDifference < 0){
                turnCounterClockwise(0.3);
            }else if (angleDifference > 0){
                turnClockwise(0.3);
            }
        }
        completeStop();
    }
}