package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

class elevatorPositions{
    public static int high   = 20500;
    public static int middle = 10000;
    public static int low    = 1000 ;
    public static int bottom = 0    ;
}


class DriveController {
    /** GENERAL CONSTANTS */
    private final double sq2 = Math.sqrt(2);

    /** SERVO CONSTANTS */
    private final double grabberOut = 0.26;
    private final double grabberMiddle = 0.79;
    private final double grabberIn  = 0.92;

    private final double[] grabberPile = {0.41, 0.38, 0.34, 0.31, 0.28};

    private final double armOut = 0.61;
    private final double armIn  = 0.41;

    private final double placerIn  = 0.01;

    private final double placerOutAutonomous = 0.76;
    private final double placerOutTeleOp = 0.85; // 0.76

    private final double pufferInit    = 0.14;
    private final double pufferGrab    = 0.36;
    private final double pufferRelease = 0.14;

    private final double grabberGrab = 0.775;
    private final double grabberOpen = 0.45 ;

    /** SENSOR CONSTANTS */
    private final double grabberCatchTrigger = 8.5;

    /** SENSORS */
    public final BNO055IMU imu;
    private final DistanceSensor grabberSensor;
    TouchSensor armSensor;

    /** TIME KEEPING VARIABLES */
    final private ElapsedTime armTime;
    final private ElapsedTime elevatorTime;

    /** SERVOS */
    final private Servo puffer ;
    final private Servo grabber;

    final private Servo placerRight;
    final private Servo placerLeft ;

    final private Servo armRight;
    final private Servo armLeft ;

    final private Servo grabberRight;
    final private Servo grabberLeft ;

    /** DRIVING MOTORS */
    final private DcMotor frontRight;
    final private DcMotor frontLeft ;
    final private DcMotor backRight ;
    final private DcMotor backLeft  ;

    /** DRIVING STATE KEEPERS */
    private double frontRightPower;
    private double frontLeftPower ;
    private double backRightPower ;
    private double backLeftPower  ;

    public double overallDrivingPower;

    /** ELEVATOR MOTORS */
    final private DcMotorEx elevatorLeft ;
    final private DcMotorEx elevatorRight;

    /** ELEVATOR STATE KEEPERS */
    private int elevatorPosition;
    private double elevatorPower;
    private boolean stopElevator;


    public DriveController(HardwareMap hm) {
        /** TIME KEEPING VARIABLES */
        // initialize the time keeping variables
        armTime      = new ElapsedTime();
        elevatorTime = new ElapsedTime();

        /** SENSORS */
        // getting the imu from the hardware map
        imu = hm.get(BNO055IMU.class, "imu");

        // initializing the imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 10);

        // getting the grabber sensor
        grabberSensor = hm.get(DistanceSensor.class, "grabberSensor");
        // getting the arm sensor
        armSensor = hm.get(TouchSensor.class, "armSensor");

        /** SERVOS */
        // getting the edge units
        puffer  = hm.servo.get("puffer" );
        grabber = hm.servo.get("grabber");

        // setting the edge units to their initial positions
        grabber.setPosition(grabberOpen);
        puffer .setPosition(pufferInit );


        // getting the placers, arms and grabbers
        placerRight = hm.servo.get("placerRight");
        placerLeft  = hm.servo.get("placerLeft" );

        armRight = hm.servo.get("armRight");
        armLeft  = hm.servo.get("armLeft" );

        grabberRight = hm.servo.get("grabberRight");
        grabberLeft  = hm.servo.get("grabberLeft" );

        // flipping the right servos direction
        armRight    .setDirection(Servo.Direction.REVERSE);
        placerRight .setDirection(Servo.Direction.REVERSE);
        grabberRight.setDirection(Servo.Direction.REVERSE);

        // setting the servos position to their initial positions
        setArmPosition(armIn);
        setPlacerPosition(placerIn);
        setGrabberPosition(grabberIn);


        /** MOTORS */
        // getting the driving motors
        frontRight = hm.dcMotor.get("frontRight");
        frontLeft  = hm.dcMotor.get("frontLeft" );
        backRight  = hm.dcMotor.get("backRight" );
        backLeft   = hm.dcMotor.get("backLeft"  );

        // flipping the two left driving motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft .setDirection(DcMotorSimple.Direction.REVERSE);

        // setting the default driving state
        frontRightPower = 0;
        frontLeftPower  = 0;
        backRightPower  = 0;
        backLeftPower   = 0;

        overallDrivingPower = 1;

        // getting the elevator motors
        elevatorLeft  = (DcMotorEx) hm.dcMotor.get("elevatorLeft" );
        elevatorRight = (DcMotorEx) hm.dcMotor.get("elevatorRight");

        // flipping the right elevator motor
        elevatorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // set the left elevator motor to use the encoder
        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // setting the default elevator state
        elevatorPosition = 0;
        elevatorPower    = 0;
        stopElevator = false;
    }

    /** IN PROGRESS */
    public void moveArm(double position) {
        // temporary
        if (armTime.seconds() > 0.2 && armTime.seconds() < 0.8 && grabberSensor.getDistance(DistanceUnit.CM) < grabberCatchTrigger) {
            setArmPosition(armIn);
        } else if (armTime.seconds() > 0.8 || grabberSensor.getDistance(DistanceUnit.CM) > grabberCatchTrigger) {
            if (position > 0) {
                grabber.setPosition(grabberOpen);
                setGrabberPosition(grabberOut);
            } else {
                setGrabberPosition(grabberIn);
            }
            setArmPosition(position * (armOut - armIn) + armIn);
        }

        if (grabberSensor.getDistance(DistanceUnit.CM) < grabberCatchTrigger && grabberLeft.getPosition() == grabberOut && grabber.getPosition() == grabberOpen) {
            grabber.setPosition(grabberGrab);
            armTime.reset();
        }
    }

    public void setElevatorPosition(int position) {
        if (position == elevatorPositions.bottom && elevatorPosition != position) {
            puffer.setPosition(pufferInit);
            setPlacerPosition(placerIn);
        } else if (position != elevatorPositions.bottom && elevatorPosition == elevatorPositions.bottom) {
            puffer.setPosition(pufferGrab);
            grabber.setPosition(grabberOpen);

            elevatorTime.reset();
            stopElevator = true;
        }
        elevatorPosition = position;
    }

    public void scoreCone() {
        puffer.setPosition(pufferRelease);
    }

    public void update() {
        // getting the biggest of the driving powers
        double biggest = Math.max(
                Math.max
                        (
                                Math.abs(this.frontRightPower),
                                Math.abs(this.backRightPower)
                        ),
                Math.max
                        (
                                Math.abs(this.frontLeftPower),
                                Math.abs(this.backLeftPower)
                        )
        );

        // normalizing the powers to be at most 1
        if (biggest > 1) {
            this.frontRightPower /= biggest;
            this.backRightPower  /= biggest;
            this.frontLeftPower  /= biggest;
            this.backLeftPower   /= biggest;
        }

        // set the driving powers into the driving motors with consideration of the overall driving power
        this.frontRight.setPower(this.frontRightPower * overallDrivingPower);
        this.frontLeft .setPower(this.frontLeftPower  * overallDrivingPower);
        this.backRight .setPower(this.backRightPower  * overallDrivingPower);
        this.backLeft  .setPower(this.backLeftPower   * overallDrivingPower);

        // change the elevator power if the elevator is not paused
        if (!stopElevator) elevatorPower = Math.max(Math.min((elevatorPosition - elevatorLeft.getCurrentPosition()) / 2300.0, 1), -1);

        // make the elevator weaker when coming down to compensate for gravity
        if (elevatorPower < 0) elevatorPower /= 2;

        // set the elevator power into the elevator motors
        this.elevatorLeft.setPower(elevatorPower);
        this.elevatorRight.setPower(elevatorPower);

        // reset the motors power to avoid accidents
        frontRightPower = 0;
        frontLeftPower  = 0;
        backRightPower  = 0;
        backLeftPower   = 0;

        overallDrivingPower = 1;

        elevatorPower = 0;

        // temporary
        if (elevatorTime.seconds() < 1 && elevatorTime.seconds() > 0.3) {
            if (elevatorPosition / 2 < elevatorLeft.getCurrentPosition() && elevatorLeft.getCurrentPosition() < elevatorPosition) {
                setPlacerPosition(placerOutTeleOp);
            }
            stopElevator = false;
        }
    }

    public void cycle() {
        try {
            for (int i = 0; i < 5; i++){
                score();

                setGrabberPosition(grabberPile[i]);

                Thread.sleep(1500);

                setArmPosition(armOut);


                while (grabberSensor.getDistance(DistanceUnit.CM) < 6){}

                Thread.sleep(750);

                grabber.setPosition(grabberGrab);

                Thread.sleep(1500);

                setGrabberPosition(grabberMiddle);

                Thread.sleep(750);

                setArmPosition(armIn);

                while (!armSensor.isPressed()){}

                Thread.sleep(750);

                setGrabberPosition(grabberIn);

                Thread.sleep(750); // replacing the sensor for now

                grabber.setPosition(grabberOpen);

                Thread.sleep(750);
            }

            score();
        } catch (InterruptedException e) {}
    }

    public void elevatorController(){
        while (true){
            elevatorPower = Math.max(Math.min((elevatorPosition - elevatorLeft.getCurrentPosition()) / 2300.0, 1), -1);

            // make the elevator weaker when coming down to compensate for gravity
            if (elevatorPower < 0) elevatorPower /= 2;

            // set the elevator power into the elevator motors
            this.elevatorLeft.setPower(elevatorPower);
            this.elevatorRight.setPower(elevatorPower);
        }
    }

    public void score(){
        try {
            puffer.setPosition(pufferGrab);
            elevatorPosition = elevatorPositions.high;

            Thread.sleep(750);

            setPlacerPosition(placerOutAutonomous);

            Thread.sleep(750);

            puffer.setPosition(pufferRelease);

            Thread.sleep(750);

            puffer.setPosition(pufferInit);
            setPlacerPosition(placerIn);
            elevatorPosition = elevatorPositions.bottom;
        }catch (InterruptedException e){}

    }


    /** DONE */
    public void mecanumDrive(Vector direction, double turningSpeed) {
        // using my equations to calculate the power ratios
        double A = (direction.y - direction.x) / sq2;
        double B = (direction.y + direction.x) / sq2;

        // setting the powers in consideration of the turning speed
        setDrivingPower(A - turningSpeed, B + turningSpeed, B - turningSpeed, A + turningSpeed);
    }

    public double getRobotAngle() {
        // get the angle from the imu
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        // normalize the angle
        if (angle < 0) angle += Math.PI * 2;
        angle = Math.PI * 2 - angle;

        return angle;
    }

    public void setArmPosition(double position) {
        armLeft .setPosition(position);
        armRight.setPosition(position);
    }

    public void setPlacerPosition(double position) {
        placerLeft .setPosition(position);
        placerRight.setPosition(position);
    }

    public void setGrabberPosition(double position) {
        grabberLeft .setPosition(position);
        grabberRight.setPosition(position);
    }

    public void setDrivingPower(double frontRightPower, double frontLeftPower, double backRightPower, double backLeftPower) {
        this.frontRightPower = frontRightPower;
        this.frontLeftPower  = frontLeftPower ;
        this.backRightPower  = backRightPower ;
        this.backLeftPower   = backLeftPower  ;
    }
}


class Vector {
    public double x;
    public double y;

    public Vector(double x, double y){
        this.x = x;
        this.y = y;
    }

    public double getAngle(){
        double a = Math.PI * 2;
        if (y < 0) a += Math.PI + Math.atan(-x / y);
        else if (y == 0){
            if (x < 0) a += Math.PI * 0.5;
            else       a += Math.PI * 1.5;
        }else{
            a += Math.atan(-x / y);
        }
        a = Math.PI * 2 - a % (Math.PI * 2);

        return a;
    }
    public double getLength(){
        return Math.sqrt(Math.pow(this.x, 2) + Math.pow(this.y, 2));
    }

    public void setAngle(double a){
        double l = this.getLength();
        this.x = Math.sin(a) * l;
        this.y = Math.cos(a) * l;
    }
    public void setLength(double r){
        double a = this.getAngle();
        this.x = Math.sin(a) * r;
        this.y = Math.cos(a) * r;
    }

    public void addAngle(double angle){
        this.setAngle(this.getAngle() + angle);
    }
}