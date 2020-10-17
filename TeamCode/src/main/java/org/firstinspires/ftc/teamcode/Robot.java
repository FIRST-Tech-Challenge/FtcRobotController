package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

public class Robot {
    DriveController driveController;
    BNO055IMU imu;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    OpMode opMode;

    //SENSORS
//    DistanceSensor frontRangeSensor;
    //   DistanceSensor backRangeSensor;

    //BULK DATA (RevExtensions2)
    RevBulkData bulkData1;
   // RevBulkData bulkData2;
    ExpansionHubEx expansionHub1;
   // ExpansionHubEx expansionHub2;

    //data logger
    DataLogger dataLogger;

    SCARAController controller;

    public Robot(OpMode opMode, Position startingPosition, boolean isAuto, boolean debuggingMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;
        driveController = new DriveController(this, startingPosition, debuggingMode);
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu 1");


        //bulk data
        expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Control Hub");
        //expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        bulkData1 = expansionHub1.getBulkInputData();
        //bulkData2 = expansionHub2.getBulkInputData();

        dataLogger = new DataLogger("UltimateGoalRobot");
    }

    //defaults to debugging mode off, starting position of 0, 0
    public Robot(OpMode opMode, boolean isAuto) {
        this(opMode, new Position(0, 0, new Angle(0, Angle.AngleType.ZERO_TO_360_HEADING)), isAuto, false);
    }

    //defaults to starting position of 0, 0
    public Robot(OpMode opMode, boolean isAuto, boolean debuggingMode) {
        this(opMode, new Position(0, 0, new Angle(0, Angle.AngleType.ZERO_TO_360_HEADING)), isAuto, debuggingMode);
    }

    public void updateBulkData() {
        bulkData1 = expansionHub1.getBulkInputData();
        //bulkData2 = expansionHub2.getBulkInputData();
    }

    public void initIMU() {
        //this.IMUReversed = reversed;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(parameters);
    }

    //public void initIMU () { initIMU(IMUReversed); }

    public Angle getRobotHeading() {
        //heading is of NEG_180_TO_180_HEADING type by default (no need for conversion)
        double heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

//        if (IMUReversed) {
//            return new Angle(heading-180, Angle.AngleType.NEG_180_TO_180_HEADING);
//        }
        //todo: check if heading should be negative or not
        return new Angle(-heading, Angle.AngleType.NEG_180_TO_180_HEADING);
    }

    public double getRobotHeadingDouble(Angle.AngleType type) {
        return getRobotHeading().convertAngle(type).getAngle();
    }

    //SETUP METHODS
    public void setupMotor(DcMotor motor, DcMotor.Direction direction) {
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //takes RunMode parameter
    public void setupMotor(DcMotor motor, DcMotor.Direction direction, DcMotor.RunMode RUN_MODE) {
        motor.setDirection(direction);
        motor.setMode(RUN_MODE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //ANGLE METHODS
//    public Orientation getAngleOrientation() { return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); }
//    public double getCurrentAngleZ() { return getAngleOrientation().firstAngle; }
//    public double getCurrentAngleY() { return getAngleOrientation().secondAngle; }
//    public double getCurrentAngleX() { return getAngleOrientation().thirdAngle; }
//    public static double getZAngle(Orientation o) { return o.firstAngle; }
//    public static double getYAngle(Orientation o) { return o.secondAngle; }
//    public static double getXAngle(Orientation o) { return o.thirdAngle; }
//    public static double wrapAngle(double angle) { return angle < 0 ? angle % (2 * Math.PI) + 2 * Math.PI : angle % (2 * Math.PI); }

    //SERVOS
    /*
    public void intakeServoOpen(){
        moveServo(intakeServo1, 0);
        moveServo(intakeServo2, 1);
    }
    public void intakeServoClose(){
        moveServo(intakeServo1, 1);
        moveServo(intakeServo2, 0);
    }
    */

    //MOTORS
    public void moveMotor(DcMotor motor, double power) {
        motor.setPower(power);
    }

}
class Constants {

    //TODO Define these constants
    final static double WHEEL_DIAMETER = 0, TICKS_PER_ROTATION = 0;
    final static double INCHES_PER_ROTATION = Math.PI * WHEEL_DIAMETER;
    final static double TICKS_PER_INCH = TICKS_PER_ROTATION / INCHES_PER_ROTATION;
    final static double DEGREES_THRESHOLD = 2;

    //TODO FINISH THESE CONSTANTS

}
