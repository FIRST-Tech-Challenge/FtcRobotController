package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.sql.Time;

class elevatorPositions{
    public static int high   = 18800;
    public static int middle = 12000;
    public static int low    = 5000 ;
    public static int bottom = 0    ;
}

class RobotController {
    /** GENERAL CONSTANTS */
    private final double sq2 = Math.sqrt(2);
    private final Telemetry telemetry;

    /** THREADS */
    public final Thread elevatorController;
    public final Thread driveController;
    public final Thread cycleController;

    private final Thread score;
    public final Thread autoCycle;

    /** SERVO CONSTANTS */
    private final double grabberMiddle = 0.26;
    public final double grabberIn  = 0.1;

    //                                  low > > > > > > > > > > > high
    public final double[] grabberPile = {0.77, 0.73, 0.7, 0.66, 0.6};

    private final double armOut = 0.89; // hiTech value 0.61;
    private final double armIn  = 0.5;  // hiTech value 0.41;

    private final double placerIn  = 0.01;

    private final double placerOutAutonomous = 0.76;
    private final double placerOutTeleOp = 0.85;

    private final double pufferGrab    = 0.18;
    private final double pufferRelease = 0.018;

    private final double grabberGrab = 0.58;
    private final double grabberOpen = 0.36;

    /** SENSOR CONSTANTS */
    private final double grabberCatchTrigger = 8.5;

    /** SENSORS */
    private final BNO055IMU imu;
    private final DistanceSensor grabberSensor;
    private final TouchSensor armSensor;
    private final TouchSensor elevatorSensor;

    /** SERVOS */
    private final Servo puffer ;
    private final Servo grabber;

    private final Servo placerRight;
    private final Servo placerLeft ;

    private final Servo armRight;
    private final Servo armLeft ;

    private final Servo grabberRight;
    private final Servo grabberLeft ;

    /** DRIVING MOTORS */
    private final DcMotor frontRight;
    private final DcMotor frontLeft ;
    private final DcMotor backRight ;
    private final DcMotor backLeft  ;

    /** DRIVING STATE KEEPERS */
    public double overallDrivingPower;

    /** ELEVATOR MOTORS */
    private final DcMotorEx elevatorLeft ;
    private final DcMotorEx elevatorRight;

    /** ELEVATOR STATE KEEPERS */
    int elevatorPosition;
    double elevatorPower;

    /** DRIVE CONTROLLER */
    Vector joystick_left;
    double A, B;


    public RobotController(HardwareMap hm, Telemetry t) {
        this.telemetry = t;
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
        elevatorSensor = hm.get(TouchSensor.class, "elevatorSensor");

        /** SERVOS */
        // getting the edge units
        puffer  = hm.servo.get("puffer" );
        grabber = hm.servo.get("grabber");

        // setting the edge units to their initial positions
        grabber.setPosition(grabberOpen);
        puffer .setPosition(pufferRelease);


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
        grabberLeft .setDirection(Servo.Direction.REVERSE);

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

        score = new Thread(() -> {
            if (elevatorPosition != elevatorPositions.bottom) {
//                telemetry.addData("elevator height", elevatorPosition);
                try {
                    puffer.setPosition(pufferGrab);

                    Thread.sleep(750);

                    setPlacerPosition(placerOutTeleOp);

                    while (gamepad.left_trigger == 0) { if (gamepad.isStopRequested) throw new InterruptedException();}

                    puffer.setPosition(pufferRelease);

                    while (gamepad.left_trigger > 0) { if (gamepad.isStopRequested) throw new InterruptedException();}

                    setPlacerPosition(placerIn);
                    elevatorPosition = elevatorPositions.bottom;

                } catch (InterruptedException e) {
                }
            } else {
//                telemetry.addData("elevator height", "not specified");
            }
//            telemetry.update();
        });

        autoCycle = new Thread(() -> {
            score(4);
            for (int i = 4; i > 0; i--){
                collect(i);
                score(i - 1);
            }

            score();
        });


        cycleController = new Thread(() -> {
            gamepad.isCycleControllerActive = true;
            while (gamepad.isCycleControllerActive) {
                if (gamepad.right_trigger > 0 || gamepad.right_bumper) {

                    // bring the grabber down
                    setGrabberPosition(grabberPile[0]);

                    // bring the grabber out the correct amount
                    setArmPosition(gamepad.right_trigger * (armOut - armIn) + armIn);

                    // catch the cone if its in range
                    if (grabberSensor.getDistance(DistanceUnit.CM) < grabberCatchTrigger)
                        grabber.setPosition(grabberGrab);
                    else grabber.setPosition(grabberOpen);

                } else {
                    if (armSensor.isPressed()) {
                        setGrabberPosition(grabberIn);
                    } else {
                        setArmPosition(armIn);
                    }
                }

                if (!score.isAlive()) {
                    if (A_pressed()) {
                        elevatorPosition = elevatorPositions.high;
                        score.start();
                    } else if (X_pressed()) {
                        elevatorPosition = elevatorPositions.middle;
                        score.start();
                    } else if (Y_pressed()) {
                        elevatorPosition = elevatorPositions.low;
                        score.start();
                    }
                }
            }
        });

        elevatorPosition = elevatorPositions.bottom;
        elevatorPower = 0;
        elevatorController = new Thread(() -> {
            gamepad.isElevatorControllerActive = true;
            while (gamepad.isElevatorControllerActive){
                elevatorPower = Math.max(Math.min((elevatorPosition - elevatorLeft.getCurrentPosition()) / 2300.0, 1), -1);

                // make the elevator weaker when coming down to compensate for gravity
                if (elevatorPower < 0) elevatorPower /= 2;

                // set the elevator power into the elevator motors
                elevatorLeft .setPower(elevatorPower);
                elevatorRight.setPower(elevatorPower);
            }
        });

        joystick_left = new Vector(0, 0);
        driveController = new Thread(() ->{
            gamepad.isDriveControllerActive = true;
            while(gamepad.isDriveControllerActive) {
                joystick_left.x = gamepad.left_stick_x;
                joystick_left.y = -gamepad.left_stick_y;

                // make the bot field oriented while considering the starting angle
                joystick_left.addAngle(-getRobotAngle() - AutonomousDrive.lastAngle);


                // using my equations to calculate the power ratios
                A = (joystick_left.y - joystick_left.x) / sq2;
                B = (joystick_left.y + joystick_left.x) / sq2;

                // slow mode if the grabber is out
                if (grabberLeft.getPosition() == grabberPile[0]) {
                    overallDrivingPower = 0.4;
                } else {
                    overallDrivingPower = 1;
                }
                // setting the powers in consideration of the turning speed
                setDrivingPower((A - gamepad.right_stick_x * 0.7) * overallDrivingPower,
                        (B + gamepad.right_stick_x * 0.7) * overallDrivingPower,
                        (B - gamepad.right_stick_x * 0.7) * overallDrivingPower,
                        (A + gamepad.right_stick_x * 0.7) * overallDrivingPower);

                telemetry.addData("left_trigger", gamepad.left_trigger);
                telemetry.addData("left_bumper", gamepad.left_bumper);
                telemetry.addData("right_trigger", gamepad.right_trigger);
                telemetry.addData("right_bumper", gamepad.right_bumper);

                telemetry.addData("a", gamepad.a);
                telemetry.addData("b", gamepad.b);
                telemetry.addData("x", gamepad.x);
                telemetry.addData("y", gamepad.y);

                telemetry.addData("left_stick_y", gamepad.left_stick_y);
                telemetry.addData("left_stick_x", gamepad.left_stick_x);
                telemetry.addData("right_stick_x", gamepad.right_stick_x);
                telemetry.addData("position", elevatorPosition);
                telemetry.addData("current position", elevatorLeft.getCurrentPosition());


                telemetry.update();
            }
        });
    }

    private
    private boolean safeSleep(int millisecond){
        while (!gamepad.isStopRequested)
    }

    private boolean a = true;
    private boolean A_pressed(){
        if (gamepad.a){
            if(a){
                a = false;
                return true;
            }
        } else a = true;
        return false;
    }

    private boolean x = true;
    private boolean X_pressed(){
        if (gamepad.x){
            if(x){
                x = false;
                return true;
            }
        } else x = true;
        return false;
    }

    private boolean y = true;
    private boolean Y_pressed(){
        if (gamepad.y){
            if(y){
                y = false;
                return true;
            }
        } else y = true;
        return false;
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

    public void score(int nextConeNum) { score(true, nextConeNum); }
    public void score() { score(false, 0); }

    public void score(boolean prepareForNext, int nextConeNum){
        try {
            if (grabber.getPosition() != grabberOpen){
                grabber.setPosition(grabberOpen);
                Thread.sleep(750);
            }
            puffer.setPosition(pufferGrab);
            elevatorPosition = elevatorPositions.high;

            Thread.sleep(750);

            setPlacerPosition(placerOutAutonomous);

            if (prepareForNext) {
                setGrabberPosition(grabberPile[nextConeNum]);

                setArmPosition(armOut * 0.8);
            }

            Thread.sleep(750);

            puffer.setPosition(pufferRelease);

            Thread.sleep(750);

            puffer.setPosition(pufferRelease);
            setPlacerPosition(placerIn);
            elevatorPosition = elevatorPositions.bottom;
        }catch (InterruptedException e){}

    }

    public void collect(int coneNum){
        try {
            setGrabberPosition(grabberPile[coneNum]);
            Thread.sleep((int)(500 / (grabberIn - grabberPile[0]) * Math.abs(grabberPile[coneNum] - grabberLeft.getPosition())));

            setArmPosition(armOut);

            while (grabberSensor.getDistance(DistanceUnit.CM) < 6) {}

            Thread.sleep(750);

            grabber.setPosition(grabberGrab);

            Thread.sleep(1500);

            setGrabberPosition(grabberMiddle);

            Thread.sleep(750);

            setArmPosition(armIn);

            while (!armSensor.isPressed()) {
            }

            Thread.sleep(750);

            setGrabberPosition(grabberIn);

            Thread.sleep(750); // replacing the sensor for now
        }catch (InterruptedException e){}
    }

    public void terminate(){
        elevatorController.interrupt();
        driveController.interrupt();
        cycleController.interrupt();
        score.interrupt();
        autoCycle.interrupt();

        elevatorPosition = elevatorPositions.bottom;

        gamepad.a = false;
        gamepad.b = false;
        gamepad.y = false;
        gamepad.x = false;

        gamepad.left_trigger = 0;
        gamepad.right_trigger = 0;

        gamepad.left_bumper = false;
        gamepad.right_bumper = false;

        gamepad.left_stick_y = 0;
        gamepad.left_stick_x = 0;
        gamepad.right_stick_x = 0;

        gamepad.isDriveControllerActive = false;
        gamepad.isElevatorControllerActive = false;
        gamepad.isCycleControllerActive = false;


    }

}


class Vector {
    public double x;
    public double y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getAngle() {
        double a = Math.PI * 2;
        if (y < 0) a += Math.PI + Math.atan(-x / y);
        else if (y == 0) {
            if (x < 0) a += Math.PI * 0.5;
            else a += Math.PI * 1.5;
        } else {
            a += Math.atan(-x / y);
        }
        a = Math.PI * 2 - a % (Math.PI * 2);

        return a;
    }

    public double getLength() {
        return Math.sqrt(Math.pow(this.x, 2) + Math.pow(this.y, 2));
    }

    public void setAngle(double a) {
        double l = this.getLength();
        this.x = Math.sin(a) * l;
        this.y = Math.cos(a) * l;
    }

    public void setLength(double r) {
        double a = this.getAngle();
        this.x = Math.sin(a) * r;
        this.y = Math.cos(a) * r;
    }

    public void addAngle(double angle) {
        this.setAngle(this.getAngle() + angle);
    }
}