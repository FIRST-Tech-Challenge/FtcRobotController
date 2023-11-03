package org.firstinspires.ftc.teamcode.Core;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Mat;

/**
 * This class contains all the hardware components that are programmed on our robot and are mapped to the robot as well.
 * Other variables like Telemetry and ElapsedTime are also created.
 */

public class HWMap {
    // Drive Motors
    private Motor leftFrontMotor;
    private Motor leftBackMotor;
    private Motor rightBackMotor;
    private Motor rightFrontMotor;

    // Mechanism Motors
    private Motor linearSlidesRight;
    private Motor linearSlidesLeft;
    private Motor intakeMotor;
    //IMU
    private static BNO055IMU imu;
    private static double imuAngle;

    //Servos
    private Servo outakeServoLeft;
    private Servo outakeServoRight;
    private CRServo axonServoLeft;
    private CRServo axonServoRight;
    private AnalogInput axonAnalogLeft;
    private AnalogInput axonAnalogRight;
    private Servo OdoRetractionLeft;
    private Servo OdoRetractionRight;
    private Servo OdoRetractionMiddle;

    //Sensors

    private ColorSensor trayLeftCS;
    private ColorSensor trayRightCS;
    private DistanceSensor distanceSensorLeft;
    private DistanceSensor distanceSensorRight;

    private final Telemetry telemetry;

    public final double servoOpen = 1.0;
    public final double servoClose = 0.0;

    public HWMap(Telemetry telemetry, HardwareMap hardwareMap) {
        //Other Variables
        this.telemetry = telemetry;


        //Drive Motors
        rightFrontMotor = new Motor(hardwareMap, "RF", Motor.GoBILDA.RPM_435); //CH Port 0
        leftFrontMotor = new Motor(hardwareMap, "LF", Motor.GoBILDA.RPM_435);//CH Port 1. The right odo pod accesses this motor's encoder port
        leftBackMotor = new Motor(hardwareMap, "LB", Motor.GoBILDA.RPM_435); //CH Port 2. The perpendicular odo pod accesses this motor's encoder port
        rightBackMotor = new Motor(hardwareMap, "RB", Motor.GoBILDA.RPM_435);//CH Port 3. The left odo pod accesses this motor's encoder port.


        //Linear Slides Motors
        linearSlidesLeft = new Motor(hardwareMap, "LSL", Motor.GoBILDA.RPM_435); //EH Port 2
        linearSlidesRight = new Motor(hardwareMap, "LSR", Motor.GoBILDA.RPM_435);//EH Port 3

        // Intake Motor
        intakeMotor = new Motor(hardwareMap, "IM", Motor.GoBILDA.RPM_435); //EH Port 0

        //IMU mapped and initialized in SampleMecanumDrive - CH 12C BUS 0
        imu = hardwareMap.get(BNO055IMU.class, "imu");


        //Outake Servos
        outakeServoLeft = hardwareMap.get(Servo.class, "OSL"); //EH Port 4
        outakeServoRight = hardwareMap.get(Servo.class, "OSR");//EH Port 5

        //ODO retraction Servos
        OdoRetractionLeft = hardwareMap.get(Servo.class, "ORL"); //CH Port 0
        OdoRetractionRight = hardwareMap.get(Servo.class, "ORR");//CH Port 1
        OdoRetractionMiddle = hardwareMap.get(Servo.class, "ORM");//CH Port 2

        //Linear Slides Servos
        axonServoLeft = new CRServo(hardwareMap, "ASL");//EH Port 0
        axonServoRight = new CRServo(hardwareMap, "ASR");//EH Port 1
        axonAnalogLeft = hardwareMap.get(AnalogInput.class, "AAL"); //EH Port 0
        axonAnalogRight = hardwareMap.get(AnalogInput.class, "AAR"); //EH Port 2

        axonServoLeft.setInverted(false);//Counterclockwise
        axonServoRight.setInverted(false);//Clockwise

        //Mapping Sensors
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "DSL");//EH Port 2
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "DSR");//EH Port 0
        trayLeftCS = hardwareMap.get(ColorSensor.class, "TLCS");//CH Port 2
        trayRightCS = hardwareMap.get(ColorSensor.class, "TRCS");//CH Port 1


        //Set Motor Direction
        leftFrontMotor.setInverted(true);
        leftBackMotor.setInverted(true);

        //Zero Power Behavior
        leftBackMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //Set Motor Mode
        leftBackMotor.setRunMode(Motor.RunMode.RawPower);
        rightBackMotor.setRunMode(Motor.RunMode.RawPower);
        leftFrontMotor.setRunMode(Motor.RunMode.RawPower);
        rightFrontMotor.setRunMode(Motor.RunMode.RawPower);

        linearSlidesRight.setRunMode(Motor.RunMode.PositionControl);
        linearSlidesLeft.setRunMode(Motor.RunMode.PositionControl);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);
    }

    @SuppressLint("DefaultLocale")
    public void Telemetry() {
        telemetry.addData("DSL: ", getDistanceSensorLeft().getDistance(DistanceUnit.MM));
        telemetry.addData("DSR: ", getDistanceSensorRight().getDistance(DistanceUnit.MM));
        telemetry.addLine(String.format("Color Sensor Left - R: %d G: %d B: %d", getTrayLeftCS().red(), getTrayLeftCS().blue(), getTrayLeftCS().green()));
        telemetry.addLine(String.format("Color Sensor Right - R: %d G: %d B: %d", getTrayRightCS().red(), getTrayRightCS().blue(), getTrayRightCS().green()));
        telemetry.addData("OPL: ", getOdoReadingLeft());
        telemetry.addData("OPP: ", getOdoReadingPerpendicular());
        telemetry.addData("OPR: ", getOdoReadingRight());
        telemetry.update();
    }

    public static double readFromIMU() {
        imuAngle = -imu.getAngularOrientation().firstAngle;
        return imuAngle;
    }

    public static void initializeIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public void open(Servo servo) {
        servo.setPosition(servoOpen);
    }

    public void close(Servo servo) {
        servo.setPosition(servoClose);
    }

    public void loop() {
        Telemetry();
    }

    public int voltsToDeg(AnalogInput servoEncoder) {
        return (int) (servoEncoder.getVoltage() / 3.3 * 360);
    }

    public Servo getOdoRetractionRight() {
        return OdoRetractionRight;
    }

    public DistanceSensor getDistanceSensorRight() {
        return distanceSensorRight;
    }

    public DistanceSensor getDistanceSensorLeft() {
        return distanceSensorLeft;
    }

    public ColorSensor getTrayRightCS() {
        return trayRightCS;
    }

    public ColorSensor getTrayLeftCS() {
        return trayLeftCS;
    }

    public Servo getOdoRetractionLeft() {
        return OdoRetractionLeft;
    }

    public CRServo getAxonServoRight() {
        return axonServoRight;
    }

    public CRServo getAxonServoLeft() {
        return axonServoLeft;
    }

    public Servo getOutakeServoRight() {
        return outakeServoRight;
    }

    public Servo getOutakeServoLeft() {
        return outakeServoLeft;
    }

    public AnalogInput getAxonAnalogRight() {
        return axonAnalogRight;
    }

    public AnalogInput getAxonAnalogLeft() {
        return axonAnalogLeft;
    }

    public Motor getLinearSlidesRight() {
        return linearSlidesRight;
    }

    public Motor getLinearSlidesLeft() {
        return linearSlidesLeft;
    }

    public Motor getIntakeMotor() {
        return intakeMotor;
    }

    public Motor getRightBackMotor() {
        return rightBackMotor;
    }

    public Motor getLeftBackMotor() {
        return leftBackMotor;
    }

    public Motor getRightFrontMotor() {
        return rightFrontMotor;
    }

    public Motor getLeftFrontMotor() {
        return leftFrontMotor;
    }

    public int getOdoReadingLeft() {
        return rightBackMotor.getCurrentPosition();
    }

    public int getOdoReadingPerpendicular() {
        return leftBackMotor.getCurrentPosition();
    }

    public int getOdoReadingRight() {
        return leftFrontMotor.getCurrentPosition();
    }



}
