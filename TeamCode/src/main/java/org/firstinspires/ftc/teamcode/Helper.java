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

class RobotController {
    /** GENERAL CONSTANTS */
    private final double sq2 = Math.sqrt(2);
    private Gamepad gamepad;

    /** SERVO CONSTANTS */
    private final double grabberOut = 0.26;
    private final double grabberMiddle = 0.79;
    private final double grabberIn  = 0.92;

    private final double[] grabberPile = {0.41, 0.38, 0.34, 0.31, 0.28};

    private final double armOut = 0.89; // hiTech value 0.61;
    private final double armIn  = 0.5;  // hiTech value 0.41;

    private final double placerIn  = 0.01;

    private final double placerOutAutonomous = 0.76;
    private final double placerOutTeleOp = 0.85;

    private final double pufferInit    = 0.14;
    private final double pufferGrab    = 0.36;
    private final double pufferRelease = 0.14;

    private final double grabberGrab = 0.775;
    private final double grabberOpen = 0.45 ;

    /** SENSOR CONSTANTS */
    private final double grabberCatchTrigger = 8.5;

    /** SENSORS */
    private final BNO055IMU imu;
    private final DistanceSensor grabberSensor;
    private final TouchSensor armSensor;

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
    public double overallDrivingPower;

    /** ELEVATOR MOTORS */
    final private DcMotorEx elevatorLeft ;
    final private DcMotorEx elevatorRight;


    public RobotController(HardwareMap hm, Gamepad gamepad) {
        this.gamepad = gamepad;
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

        // flipping the relevant servos direction
        armLeft     .setDirection(Servo.Direction.REVERSE);
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

        overallDrivingPower = 1;

        // getting the elevator motors
        elevatorLeft  = (DcMotorEx) hm.dcMotor.get("elevatorLeft" );
        elevatorRight = (DcMotorEx) hm.dcMotor.get("elevatorRight");

        // flipping the right elevator motor
        elevatorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // set the left elevator motor to use the encoder
        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void scoringController(){

    }

    public void collectingController(){


    }

    int elevatorPosition = elevatorPositions.bottom;
    double elevatorPower = 0;
    public void elevatorController(){
        elevatorPower = Math.max(Math.min((elevatorPosition - elevatorLeft.getCurrentPosition()) / 2300.0, 1), -1);

        // make the elevator weaker when coming down to compensate for gravity
        if (elevatorPower < 0) elevatorPower /= 2;

        // set the elevator power into the elevator motors
        elevatorLeft .setPower(elevatorPower);
        elevatorRight.setPower(elevatorPower);

        elevatorController();
    }

    Vector joystick_left = new Vector(0, 0);
    double A, B;
    public void driveController() {
        joystick_left.x =  gamepad.left_stick_x;
        joystick_left.y = -gamepad.left_stick_y;

        // make the bot field oriented while considering the starting angle
        joystick_left.addAngle(-getRobotAngle() - AutonomousDrive.lastAngle);


        // using my equations to calculate the power ratios
        A = (joystick_left.y - joystick_left.x) / sq2;
        B = (joystick_left.y + joystick_left.x) / sq2;

        // slow mode if the grabber is out
        if (grabberLeft.getPosition() == grabberOut) {
            overallDrivingPower = 0.4;
        } else {
            overallDrivingPower = 1;
        }
            // setting the powers in consideration of the turning speed
            setDrivingPower((A - gamepad.right_stick_x * 0.7) * overallDrivingPower,
                            (B + gamepad.right_stick_x * 0.7) * overallDrivingPower,
                            (B - gamepad.right_stick_x * 0.7) * overallDrivingPower,
                            (A + gamepad.right_stick_x * 0.7) * overallDrivingPower);
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
        frontRight.setPower(frontRightPower * overallDrivingPower);
        frontLeft .setPower(frontLeftPower  * overallDrivingPower);
        backRight .setPower(backRightPower  * overallDrivingPower);
        backLeft  .setPower(backLeftPower   * overallDrivingPower);
    }


    public void cycle() {
        try {
            for (int i = 0; i < 5; i++){
                score();

                setGrabberPosition(grabberPile[i]);

                Thread.sleep(1500);
                elevatorLeft .setPower(elevatorPower);
                elevatorRight.setPower(elevatorPower);

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
        elevatorController();
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