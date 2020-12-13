package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Main Teleop
 *
 * 3 October 2020
 */

@TeleOp(name = "MainTeleop")
public class MainTeleop extends LinearOpMode{
    private DcMotor motorFrontRight, motorFrontLeft, motorBackLeft, motorBackRight;

    private CRServo leftConveyor, rightConveyor, elevator, intake;
    private DcMotor outtakeRight, outtakeLeft;
    private Servo flipper;

    private CRServo leftIntakeServo, rightIntakeServo;

    private BNO055IMU imu;

    private IMURobot robot;

    //Figures for ring elevator calculations
    private static final double PINION_CIRCUMFERENCE = 2.57;
    private static final double ELEVATOR_HEIGHT = 5.0;
    private static final double PINION_REVOLUTIONS = ELEVATOR_HEIGHT/PINION_CIRCUMFERENCE;
    private static final double SERVO_RPM = 50.0;
    private static final double ELEVATOR_TIME = PINION_REVOLUTIONS/SERVO_RPM * 60;

    //Figures for telemetry calculations
    private static final int OUTTAKE_MOTOR_RPM = 1100;
    private static final double OUTTAKE_GEAR_RATIO = 3.0;
    private static final double OUTTAKE_WHEEL_RADIUS_IN = 2;
    private static final double OUTTAKE_WHEEL_RADIUS_M = OUTTAKE_WHEEL_RADIUS_IN*0.0254;

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
        intake = hardwareMap.crservo.get("intake");
        leftConveyor = hardwareMap.crservo.get("leftConveyor");
        rightConveyor = hardwareMap.crservo.get("rightConveyor");

        //elevator and flipper
        elevator = hardwareMap.crservo.get("elevator");
        flipper = hardwareMap.servo.get("flipper");

        //launcher
        outtakeRight = hardwareMap.dcMotor.get("outtakeRight");
        outtakeLeft = hardwareMap.dcMotor.get("outtakeLeft");

        //lifting and lowering intake
        leftIntakeServo = hardwareMap.crservo.get("LIrelease");
        rightIntakeServo = hardwareMap.crservo.get("RIrelease");

        //Encoders
        verticalLeft = hardwareMap.dcMotor.get("leftOdometry");
        verticalRight = hardwareMap.dcMotor.get("rightOdometry");
        horizontal = hardwareMap.dcMotor.get("outtakeRight");

        //Initialize imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //reverse the needed motors
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reverse one of the outtakes
        outtakeLeft.setDirection(DcMotor.Direction.REVERSE);

        robot = new IMURobot(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft,
                imu, leftIntakeServo, rightIntakeServo, leftConveyor, rightConveyor, elevator, flipper, intake,
                outtakeRight, outtakeLeft, this);

        robot.setupRobot();//calibrate IMU, set any required parameters

        double powerMod = 1.0;
        double intakeMod = 1.0;
        double outtakeMod = 0.46;//0.46

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

            //stuff to program still
            //click a button to move in position to launch at goals and (another for) power shot***
            //stuff for wobble

            //everything intake
            /*
            Change direction of intake
            */
            if(gamepad1.a){//press and hold a while running intake
                intakeMod = -1.0;
            }else{
                intakeMod = 1.0;
            }

//            //Release intake
//            if(gamepad1.x){
//                lowerIntake();
//            }
//            if(gamepad1.y){
//                raiseIntake();
//            }
//            if(gamepad1.dpad_up){
//                leftIntakeServo.setPower(0.1);
//                rightIntakeServo.setPower(0.1);
//                wait(100);
//                leftIntakeServo.setPower(0);
//                rightIntakeServo.setPower(0);
//            }
//            if(gamepad1.dpad_down){
//                leftIntakeServo.setPower(-0.1);
//                rightIntakeServo.setPower(-0.1);
//                wait(100);
//                leftIntakeServo.setPower(0);
//                rightIntakeServo.setPower(0);
//            }

            double intakeSpeed = gamepad1.left_trigger * intakeMod;
            intake.setPower(intakeSpeed);
            rightConveyor.setPower(intakeSpeed);//turn conveyor on when the intake turns on
            leftConveyor.setPower(intakeSpeed);

            //Ring elevator
            //Run by a continuous servo; run continuous servo for some amount of time
//            if(gamepad2.x){
//                raiseElevator();
//            }
//
//            if(gamepad2.y){
//                lowerElevator();
//            }
//            if(gamepad2.dpad_left){
//                elevator.setPower(-0.1);
//                wait(100);
//                elevator.setPower(0);
//            }
//            if(gamepad2.dpad_right){
//                elevator.setPower(0.1);
//                wait(100);
//                elevator.setPower(0);
//            }

            //Ring flipper
            //Run by a servo, 1 is fully "flipped" position, 0 is fully "retracted" position
            //Hold down b button to flip ring out
            while(gamepad2.b && flipper.getPosition() <= 1){
                flipper.setPosition(flipper.getPosition() + 0.01);
            }

            while(!gamepad2.b && flipper.getPosition() >= 0){
                flipper.setPosition(flipper.getPosition() - 0.01);
            }

            telemetry.addData("flipper position", flipper.getPosition());


            //everything outtake/launch
            /*
            Ability to test a variety of outtake motor speeds from 1 to 0
            */
            if(gamepad2.dpad_up){
                if(outtakeMod != 1.0){
                    outtakeMod += 0.1;
                }
            }
            if(gamepad2.dpad_down){
                if(outtakeMod != 0.0){
                    outtakeMod -= 0.1;
                }
            }
            //Sending data on power of outtake, outtake motor RPM, and tangential velocity of outtake wheel to telemetry
            double outtakePower = (gamepad2.right_trigger * outtakeMod);
            outtakeLeft.setPower(outtakePower);
            outtakeRight.setPower(outtakePower);

            double outtakeRPM = outtakePower * OUTTAKE_MOTOR_RPM * OUTTAKE_GEAR_RATIO;
            double outtakeWheelVelocity = (outtakeRPM * 2 * Math.PI * OUTTAKE_WHEEL_RADIUS_M)/60;


            //everything driving
            //Mecanum drive using trig
            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - (Math.PI/4);
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;

            double powerOne = r*Math.sin(angle);
            double powerTwo = r*Math.cos(angle);

            motorFrontLeft.setPower((powerOne - (rotation))*powerMod);
            motorFrontRight.setPower((powerTwo + (rotation))*powerMod);
            motorBackLeft.setPower((powerTwo - (rotation))*powerMod);
            motorBackRight.setPower((powerOne + (rotation))*powerMod);


            //Sending data on power of outtake, outtake motor RPM, and tangential velocity of outtake wheel to telemetry
            telemetry.addData("Outtake Power", outtakePower);
            telemetry.addData("Outtake RPM", outtakeRPM);
            telemetry.addData("Outtake Wheel Velocity (m/s)", outtakeWheelVelocity);

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

//    private void raiseElevator(){
//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//
//        while(timer.milliseconds() < 200){
//            elevator.setPower(1);
//        }
//
//        elevator.setPower(0);
//    }
//
//    private void lowerElevator(){
//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//
//        while(timer.milliseconds() < 200){
//            elevator.setPower(-1);
//        }
//
//        elevator.setPower(0);
//    }
//    private void raiseIntake(){
//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//
//        while(timer.milliseconds() < 200){
//            leftIntakeServo.setPower(1);
//            rightIntakeServo.setPower(1);
//        }
//
//        leftIntakeServo.setPower(0);
//        rightIntakeServo.setPower(0);
//    }
//    private void lowerIntake(){
//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//
//        while(timer.milliseconds() < 200){
//            leftIntakeServo.setPower(-1);
//            rightIntakeServo.setPower(-1);
//        }
//
//        leftIntakeServo.setPower(0);
//        rightIntakeServo.setPower(0);
//    }
    public void odometryNormalizeAngle(){
        while (globalPositionUpdate.returnOrientation() > 0){
            robot.turnCounterClockwise(1);
        }

        while (globalPositionUpdate.returnOrientation() < 0){
            robot.turnClockwise(1);
        }

        if (globalPositionUpdate.returnOrientation() == 0){
            robot.completeStop();
        }
    }

    public void odometryDriveToPos (double xPos, double yPos) {
        double C = 0;
        while (globalPositionUpdate.returnXCoordinate() > xPos) {
            robotStrafe(1, -90);
        }
        while (globalPositionUpdate.returnXCoordinate() < xPos) {
            robotStrafe(1, 90);
        }
        if (globalPositionUpdate.returnXCoordinate() == xPos) {
            robot.completeStop();
            odometryNormalizeAngle();
            C = 1;
        }


        while (globalPositionUpdate.returnXCoordinate() > yPos && C == 1) {
            robotStrafe(-1, 0);
        }
        while (globalPositionUpdate.returnXCoordinate() < yPos && C == 1) {
            robotStrafe(1, 0);
        }
        if (globalPositionUpdate.returnXCoordinate() < yPos && C == 1) {
            robot.completeStop();
            odometryNormalizeAngle();
            C = 2;
        }
    }
    public void robotStrafe (double power, double angle){
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