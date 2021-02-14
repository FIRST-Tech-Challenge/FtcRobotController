package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "OdometryAutoTest")
public class OdometryAutoTest extends LinearOpMode{

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

        robot = new IMURobot(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft, verticalLeft, verticalRight, horizontal,
                imu, this);

        robot.setupRobot();//calibrate IMU, set any required parameters

        waitForStart();

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        telemetry.addLine("driving to pos 1");
        telemetry.update();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        robot.driveToPos(24, 0, .5);
        Thread.sleep(2000);
        telemetry.addLine("driving to pos 2");
        telemetry.update();
        robot.driveToPos(24, 24, .5);
        Thread.sleep(2000);
        telemetry.addLine("driving to pos 3");
        telemetry.update();
        robot.driveToPos(0, 24, .5);
        Thread.sleep(2000);
        telemetry.addLine("driving to pos 4");
        telemetry.update();
        robot.driveToPos(0,0, .5);
    }

    private void odometryDriveToPos(double xPos, double yPos) throws InterruptedException{
        double C = 0;
        while ((Math.sqrt(Math.pow(xPos - (globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH),2) + Math.pow(yPos - (globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH), 2)) > 1)){
            robot.resetAngle();
            double correction = robot.getCorrection();
            if (globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH < xPos - 1 && C == 0){
                robot.correctedTankStrafe(-.5, .5, correction);
            }else if (globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH > xPos + 1 && C == 0){
                robot.correctedTankStrafe(.5, -.5, correction);
            }else if (globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH < xPos + 1 || globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH > xPos - 1){
                robot.completeStop();
                C = 1;
            }

            if (globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH < yPos - 1 && C == 1){
                robot.correctedTankStrafe(.5, .5, correction);
            }else if (globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH > yPos + 1 && C == 1){
                robot.correctedTankStrafe(-.5, -.5, correction);
            }else if (globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH < yPos + 1 || globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH > yPos - 1){
                robot.completeStop();
                C = 2;
            }
        }
    }

    private void robotStrafe (double power, double angle, double correction) throws InterruptedException{
        //restart angle tracking
        robot.resetAngle();

        //convert direction (degrees) into radians
        double newDirection = angle * Math.PI/180 + Math.PI/4;
        //calculate powers needed using direction
        double leftPower = Math.cos(newDirection) * power;
        double rightPower = Math.sin(newDirection) * power;

        //while(opMode.opModeIsActive()){
        //Get a correction
        //double correction = robot.getCorrection();
        //Use the correction to adjust robot power so robot faces straight
        robot.correctedTankStrafe(leftPower, rightPower, correction);
        //}
    }

}
