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

@TeleOp(name = "Main Odometry Teleop")
public class MainTeleopOdometry extends LinearOpMode{
    private DcMotor motorFrontRight, motorFrontLeft, motorBackLeft, motorBackRight;

    private DcMotor intake;
    private DcMotor outtakeRight, outtakeLeft, wobbleArm;
    private Servo flipper, wobbleClaw;

    private Orientation angles, lastAngles, startAngles;
    private double globalAngle;

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

        //intake and conveyor
        intake = hardwareMap.dcMotor.get("intake");

        //wobble and flipper
        wobbleArm = hardwareMap.dcMotor.get("wobbleArm");
        wobbleClaw = hardwareMap.servo.get("wobbleClaw");
        flipper = hardwareMap.servo.get("flipper");

        //launcher  //Feb 7 - Jeff commmented out these motor definitions
        outtakeRight = hardwareMap.dcMotor.get("outtakeRight");
        outtakeLeft = hardwareMap.dcMotor.get("outtakeLeft");
        //Jeff added
        //outtakeLeft=hardwareMap.get(DcMotor.class, "outtakeLeft");
        //outtakeRight=hardwareMap.get(DcMotor.class, "outtakeRight");
        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
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

        outtakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Initialize imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");


        //reverse the needed motors
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot = new IMURobot(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft,
                imu, wobbleArm, wobbleClaw, flipper, intake,
                outtakeRight, outtakeLeft, this);

        robot.setupRobot();//calibrate IMU, set any required parameters

        double powerMod;
        double wobbleMod;
        double intakeMod;


        waitForStart();

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();

        while(opModeIsActive()){

            /*
            Checks if right bumper is pressed. If so, power is reduced
             */

            if(gamepad1.right_bumper){
                powerMod = 0.5;
            }else{
                powerMod = 1.0;
            }

            //everything intake

            //changes direction of intake
            if(gamepad1.a){//press and hold a while running intake
                intakeMod = 1.0;
            }else{
                intakeMod = -1.0;
            }
            double intakeSpeed = gamepad1.left_trigger * intakeMod * .85;
            intake.setPower(intakeSpeed);

            //Ring flipper
            //Run by a servo, 1 is fully "flipped" position, 0 is fully "retracted" position
            //Hold down b button to flip ring out

            if(gamepad2.b){
                flipper.setPosition(1);
            }

            if(gamepad2.a){
                flipper.setPosition(0);
            }

            telemetry.addData("flipper position", flipper.getPosition());

            //everything shooting

            if (gamepad2.right_bumper){
                shootPowerShot();
            }
            if (gamepad2.right_trigger > 0.3){
                shootGoal();
            }

            //everything wobble

            if(gamepad2.left_bumper){
                wobbleMod = 1.0;
            }else{
                wobbleMod = .15;
            }

            wobbleArm.setPower(gamepad2.left_stick_y * wobbleMod );

            if(gamepad2.x){
                wobbleClaw.setPosition(0);
            }

            if(gamepad2.y){
                wobbleClaw.setPosition(1);
            }

            if(gamepad1.left_bumper){
                shootGoal();
            }
            if(gamepad1.b){
                shootPowerShot();
            }

            if(gamepad1.x){
                goToEnd();
            }

            //everything driving
            //Mecanum drive using trig


            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - (Math.PI / 4);
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;

            double powerOne = r * Math.sin(angle);
            double powerTwo = r * Math.cos(angle);

            motorFrontLeft.setPower((powerOne - (rotation))*powerMod);
            motorFrontRight.setPower((powerTwo + (rotation))*powerMod);
            motorBackLeft.setPower((powerTwo - (rotation))*powerMod);
            motorBackRight.setPower((powerOne + (rotation))*powerMod);

            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
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
        setOdometryAngle(direction);
        double distanceX = xPos - (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);//0
        double distanceY = yPos - (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);//0

        double angle = (Math.atan2(distanceY,distanceX)-(Math.PI/4));
        double distance = Math.hypot(distanceX,distanceY);//0

        double powerOne = 1 * Math.sin(angle);
        double powerTwo = 1 * Math.cos(angle);

        while (distance > 1.5){
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
            }


            motorFrontLeft.setPower(powerOne);
            motorFrontRight.setPower(powerTwo);
            motorBackLeft.setPower(powerTwo);
            motorBackRight.setPower(powerOne);
            telemetry.addData("Distance: ", distance);
            telemetry.addData("DistanceX: ", distanceX);
            telemetry.addData("DistanceY: ", distanceY);
            telemetry.addData("Xpos: ", globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH);
            telemetry.addData("Ypos: ", globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH);
            telemetry.update();
        }
        robot.completeStop();
        setOdometryAngle(direction);

    }

    public void odometryDriveToPosCorrected (double xPos, double yPos, double direction) {
        if (getOdometryAngleDifference(direction) > 1.5){
            setOdometryAngle(0);
        }
        double distanceX = xPos - (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);//0
        double distanceY = yPos - (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);//0

        double angle = (Math.atan2(distanceY,distanceX)-(Math.PI/4));
        double distance = Math.hypot(distanceX,distanceY);//0

        double powerOne = 1 * Math.sin(angle);
        double powerTwo = 1 * Math.cos(angle);

        double angleDifference = getOdometryAngleDifferenceNegative(direction);

        while (distance > 1.5){
            if (gamepad1.y){
                break;
            }

            angleDifference = getOdometryAngleDifferenceNegative(direction);
            double correction = angleDifference * 0.1;


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
            }


            motorFrontLeft.setPower(powerOne+correction);
            motorFrontRight.setPower(powerTwo-correction);
            motorBackLeft.setPower(powerTwo+correction);
            motorBackRight.setPower(powerOne-correction);
            telemetry.addData("Distance: ", distance);
            telemetry.addData("DistanceX: ", distanceX);
            telemetry.addData("DistanceY: ", distanceY);
            telemetry.addData("Xpos: ", globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH);
            telemetry.addData("Ypos: ", globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH);
            telemetry.update();
        }
        robot.completeStop();
    }

    public void setOdometryAngle(double desiredAngle) {

        double rawAngleDifference = getAngleRaw(desiredAngle);
        double relativeAngleDifference = getOdometryAngleDifference(desiredAngle);


        while (relativeAngleDifference > 1.5){
            if (gamepad1.y){
                break;
            }

            rawAngleDifference = getAngleRaw(desiredAngle);
            relativeAngleDifference = getOdometryAngleDifference(desiredAngle);


            if ((desiredAngle > globalPositionUpdate.returnOrientation()) && (rawAngleDifference > 180)){
                if (relativeAngleDifference > 15){
                    turnClockwise(1);
                }else if (relativeAngleDifference <= 15 && relativeAngleDifference > 4){
                    turnClockwise(0.3);
                }else if (relativeAngleDifference <= 4 && relativeAngleDifference > 2){
                    turnClockwise(0.2);
                }else{
                    break;
                }
            }else if ((desiredAngle < globalPositionUpdate.returnOrientation()) && (rawAngleDifference <= 180)){
                if (relativeAngleDifference > 15){
                    turnCounterClockwise(1);
                }else if (relativeAngleDifference <= 15 && relativeAngleDifference > 4){
                    turnCounterClockwise(0.3);
                }else if (relativeAngleDifference <= 4 && relativeAngleDifference > 2){
                    turnCounterClockwise(0.2);
                }else{
                    break;
                }
            }else if ((desiredAngle < globalPositionUpdate.returnOrientation()) && (rawAngleDifference > 180)){
                if (relativeAngleDifference > 15){
                    turnClockwise(1);
                }else if (relativeAngleDifference <= 15 && relativeAngleDifference > 4){
                    turnClockwise(0.3);
                }else if (relativeAngleDifference <= 4 && relativeAngleDifference > 2){
                    turnClockwise(0.2);
                }
                else{
                    break;
                }
            }else if ((desiredAngle > globalPositionUpdate.returnOrientation()) && (rawAngleDifference <= 180)){
                if (relativeAngleDifference > 15){
                    turnCounterClockwise(1);
                }else if (relativeAngleDifference <= 15 && relativeAngleDifference > 4){
                    turnCounterClockwise(0.3);
                }else if (relativeAngleDifference <= 4 && relativeAngleDifference > 2){
                    turnCounterClockwise(0.2);
                }else{
                    break;
                }

            }else{
                break;
            }
        }
        robot.completeStop();
    }

    public void shootPowerShot() throws InterruptedException{
        //Shot 1
        odometryDriveToPosCorrected(-35.3,54,0);
        robot.shootRingsPower();
        //Shot 2
        odometryDriveToPosCorrected(-40.6,54,0);
        robot.shootRingsPower();
        //Shot 3
        odometryDriveToPosCorrected(-48,54,0);
        robot.shootRingsPower();
    }

    public void shootGoal() throws InterruptedException{
        odometryDriveToPosCorrected(-18,54,350);
        robot.shootRings();
    }

    public void goToEnd(){
        odometryDriveToPosCorrected(-15.88, 69.8,0);
    }


    public double getOdometryAngleDifference(double desiredAngle){
        double angleDifference = Math.abs(desiredAngle - globalPositionUpdate.returnOrientation());

        if (angleDifference > 180){
            angleDifference = 360 - angleDifference;
        }

        return angleDifference;
    }

    public double getOdometryAngleDifferenceNegative(double desiredAngle){
        double angleDifference = Math.abs(desiredAngle - globalPositionUpdate.returnOrientation());

        if (angleDifference > 180){
            angleDifference = angleDifference - 360;
        }

        return angleDifference;
    }

    public double getAngleRaw (double desiredAngle){
        return ((double) Math.abs(desiredAngle - globalPositionUpdate.returnOrientation()));
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
}