package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;
//import org.openftc.revextensions2.ExpansionHubEx;

/**
 * Main Teleop
 *
 * 3 October 2020
 */

@TeleOp(name = "MainTeleop")
public class MainTeleop extends LinearOpMode{
    private DcMotor motorFrontRight, motorFrontLeft, motorBackLeft, motorBackRight;

    private CRServo leftConveyor, rightConveyor, intake;
    private DcMotor outtakeRight, outtakeLeft, wobbleArm;
    private Servo flipper, wobbleClaw;

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

        //wobble and flipper
        wobbleArm = hardwareMap.dcMotor.get("wobbleArm");
        wobbleClaw = hardwareMap.servo.get("wobbleClaw");
        flipper = hardwareMap.servo.get("flipper");

        //launcher  //Feb 7 - Jeff commmented out these motor definitions
        //outtakeRight = hardwareMap.dcMotor.get("outtakeRight");
        //outtakeLeft = hardwareMap.dcMotor.get("outtakeLeft");
        //Jeff added
        outtakeLeft=hardwareMap.get(DcMotor.class, "outtakeLeft");
        outtakeRight=hardwareMap.get(DcMotor.class, "outtakeRight");
        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Encoders
        verticalLeft = hardwareMap.dcMotor.get("FL");
        verticalRight = hardwareMap.dcMotor.get("FR");
        horizontal = hardwareMap.dcMotor.get("BL");

        //Initialize imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //voltSensor = hardwareMap.voltageSensor.get("outtakeRight");

        //reverse the needed motors
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        leftConveyor.setDirection(CRServo.Direction.REVERSE);

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        robot = new IMURobot(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft,
                imu, wobbleArm, wobbleClaw, leftConveyor, rightConveyor, flipper, intake,
                outtakeRight, outtakeLeft, this);

        robot.setupRobot();//calibrate IMU, set any required parameters

        double powerMod = 1.0;
        double intakeMod = 1.0;
        double outtakeMod = .325;
        double wobbleMod = .3;

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

            //everything intake
            /*
            Change direction of intake
            */
            if(gamepad1.a){//press and hold a while running intake
                intakeMod = -1.0;
            }else{
                intakeMod = 1.0;
            }
            double intakeSpeed = gamepad1.left_trigger * intakeMod;
            intake.setPower(intakeSpeed);
            rightConveyor.setPower(intakeSpeed);//turn conveyor on when the intake turns on
            leftConveyor.setPower(intakeSpeed);



            //Ring flipper
            //Run by a servo, 1 is fully "flipped" position, 0 is fully "retracted" position
            //Hold down b button to flip ring out


            if(gamepad2.a){
                flipper.setPosition(0);
                Thread.sleep(500);
                flipper.setPosition(1);
            }

            telemetry.addData("flipper position", flipper.getPosition());


            //everything outtake/launch

            //Sending data on power of outtake, outtake motor RPM, and tangential velocity of outtake wheel to telemetry

            if(gamepad2.right_bumper){
                outtakeMod = 0.32; //power shots
            }else{
                outtakeMod = 0.3225;
            }
            double outtakePower = (gamepad2.right_trigger * outtakeMod);
            outtakeLeft.setPower(outtakePower);
            outtakeRight.setPower(outtakePower);


            double outtakeRPM = outtakePower * OUTTAKE_MOTOR_RPM * OUTTAKE_GEAR_RATIO;
            double outtakeWheelVelocity = (outtakeRPM * 2 * Math.PI * OUTTAKE_WHEEL_RADIUS_M)/60;

            //everything wobble

            if(gamepad2.left_bumper){
                wobbleMod = 1.0;
            }else{
                wobbleMod = .3;
            }
            wobbleArm.setPower(gamepad2.left_stick_y * wobbleMod );

            if(gamepad2.x){
                wobbleClaw.setPosition(0);
            }

            if(gamepad2.y){
                wobbleClaw.setPosition(1);
            }


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
            //telemetry.addData("Volts: ", volts);

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
            //telemetry.addData("5v monitor", expansionHub.read5vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)); //Voltage from the phone
           // telemetry.addData("12v monitor", expansionHub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)); //Battery voltage
            telemetry.update();

            telemetry.update();
            idle();
        }
        globalPositionUpdate.stop();

    }


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