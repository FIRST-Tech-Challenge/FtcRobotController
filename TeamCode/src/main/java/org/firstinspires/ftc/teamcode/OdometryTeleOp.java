package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Main Teleop
 *
 * 3 October 2020
 */

@TeleOp(name = "Odometry Teleop")
public class OdometryTeleOp extends LinearOpMode{
    private DcMotor motorFrontRight, motorFrontLeft, motorBackLeft, motorBackRight;

    private BNO055IMU imu;

    private IMURobot robot;

    final double WHEEL_DIAMETER = 1.5;
    final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    final double COUNTS_PER_REVOLUTION = 1280;
    final double COUNTS_PER_INCH = COUNTS_PER_REVOLUTION/WHEEL_CIRCUMFERENCE;

    //Odometry encoder wheels
    DcMotor verticalRight, verticalLeft, horizontal;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");


        //Encoders
        verticalLeft = hardwareMap.dcMotor.get("FL");
        verticalRight = hardwareMap.dcMotor.get("FR");
        horizontal = hardwareMap.dcMotor.get("BL");

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
                imu, this);

        robot.setupRobot();//calibrate IMU, set any required parameters

        double powerMod = 1.0;
        double intakeMod = 1.0;
        double outtakeMod = 1.0;

        waitForStart();

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        while(opModeIsActive()){
            /*
            Checks if right bumper is pressed. If so, power is reduced
             */
            if(gamepad1.right_bumper){
                powerMod = 0.5;
            }else{
                powerMod = 1.0;
            }

            if(gamepad1.y){
                odometryDriveToPos(0,0);
            }

            if(gamepad1.x){
                odometryDriveToPos(0, 24);
            }

            if(gamepad1.b){
                odometryDriveToPos(24, 0);
            }

            if(gamepad1.a){
                odometryDriveToPos(24, 24);
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

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();

            telemetry.update();
            idle();
        }
        globalPositionUpdate.stop();

    }

    public void odometryNormalizeAngle() throws InterruptedException{

        while(Math.abs(globalPositionUpdate.returnOrientation()) > 5){
            if(globalPositionUpdate.returnOrientation() > 0){
                robot.turnCounterClockwise(0.1);
            }else if(globalPositionUpdate.returnOrientation() < 0){
                robot.turnClockwise(0.1);
            }else{
                break;
            }
        }
        robot.completeStop();
    }

    public void odometryDriveToPos (double xPos, double yPos) throws InterruptedException{
        double C = 0;
        while (Math.abs(globalPositionUpdate.returnXCoordinate() - xPos) > 1) {
            double angle = globalPositionUpdate.returnXCoordinate() < xPos ? 90 : -90;
            robotStrafe(.4, angle);

            if(globalPositionUpdate.returnXCoordinate() == xPos){
                break;
            }
        }
        robot.completeStop();
        odometryNormalizeAngle();
        Thread.sleep(500);


        while (Math.abs(globalPositionUpdate.returnYCoordinate() - yPos) > 1) {
            double power = globalPositionUpdate.returnYCoordinate() < yPos ? -.4 : .4;

            if(globalPositionUpdate.returnYCoordinate() == yPos){
                break;
            }
        }
        robot.completeStop();
        odometryNormalizeAngle();
        Thread.sleep(500);
    }
    public void robotStrafe (double power, double angle) throws InterruptedException{
        //restart angle tracking
        robot.resetAngle();

        //convert direction (degrees) into radians
        double newDirection = angle * Math.PI/180 + Math.PI/4;
        //calculate powers needed using direction
        double leftPower = Math.cos(newDirection) * power;
        double rightPower = Math.sin(newDirection) * power;

        //while(opMode.opModeIsActive()){
        //Get a correction
        double correction = robot.getCorrection();
        //Use the correction to adjust robot power so robot faces straight
        robot.correctedTankStrafe(leftPower, rightPower, correction);
        //}
    }
}