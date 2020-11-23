package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/*
    NOTE -conventions for the measurement of distances and anlges-
    distances will be recorded and implemented in Standard units (meters), do not use the imperial system in your code
        this is because the metric system is easier to convert and work with than imperial units
    angles will be measured in radians
        this is due to how the Trig functions operate in java (they expect input, and return, values measured in radians)
 */


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for the robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *  more motors, sensors, servos, etc will be added as needed
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 *
 *
 * the purpose of this class is to act as a centralized method for initializing the hardware on the
 * robot, so that the addition of hardware elements can be accounted for in one central area, reducing
 * the risk of error in the event the robot is updated
 */
public class HardwareUltimateGoal {
    /* Public OpMode members. */
    public DcMotor  leftDrive;
    public DcMotor  rightDrive;
    public DcMotor  flyWheel1;
    public DcMotor  flyWheel2;
    //public DcMotor  conveyor1 = null; //commented out bc it's not installed yet

    public Servo turretElevator;
    public Servo turretRotator;
    public Servo turretLauncher;

    //public TouchSensor touch1 = null; //commented out bc it's not installed yet
    //public ColorSensor color1 = null; //commented out bc it's not installed yet


    //IMU Sensor
    public BNO055IMU imu;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    /* some variables for different measurements of the robot */ //TODO: keep up to date
    public double turretHeight = 0.2023;
    public double robotWidth = 0.4572; //18" measured in meters
    // stats for the TorqueNADO motors
    public final double NADO_COUNTS_PER_MOTOR_REV = 1440;
    public final double NADO_DRIVE_GEAR_REDUCTION = 32.0/24.0;  // This is < 1.0 if geared UP (to increase speed)
    public final double NADO_WHEEL_DIAMETER_METERS= 0.1016; //(4") For figuring circumference
    public final double NADO_COUNTS_PER_METER      = (NADO_COUNTS_PER_MOTOR_REV * NADO_DRIVE_GEAR_REDUCTION) /
            (NADO_WHEEL_DIAMETER_METERS * Math.PI);
    public final double NADO_METERS_PER_COUNT = 1.0 / NADO_COUNTS_PER_METER;

    // stats for the NeveRest motor
    public final double NEVE_COUNTS_PER_MOTOR_REV = 448; // 28 * 16(gear ratio)
    public final double NEVE_DRIVE_GEAR_REDUCTION = 1;    // This is < 1.0 if geared UP
    public final double NEVE_WHEEL_DIAMETER_METERS= 0.0762; //(3") For figuring circumference
    public final double NEVE_COUNTS_PER_METER      = (NEVE_COUNTS_PER_MOTOR_REV * NEVE_DRIVE_GEAR_REDUCTION) /
            (NEVE_WHEEL_DIAMETER_METERS * Math.PI);
    public final double NEVE_METERS_PER_REV = (NEVE_COUNTS_PER_MOTOR_REV * NEVE_DRIVE_GEAR_REDUCTION) / NEVE_COUNTS_PER_METER;

    /* Constructor */
    public HardwareUltimateGoal(){

    }

    /*another constructor for testing, when there isn't an autonomous to write the heading and position files*/
    public HardwareUltimateGoal(String writesFilesAsRedTeam){
        writePositionHeading(new double[]{1.79705, -1.79705}, Math.PI/2);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive   = hwMap.get(DcMotor.class, "leftDrive");
        rightDrive  = hwMap.get(DcMotor.class, "rightDrive");
        rightDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        flyWheel1   = hwMap.get(DcMotor.class, "flywheelLeft");
        flyWheel2   = hwMap.get(DcMotor.class, "flywheelRight");
        flyWheel1.setDirection(DcMotor.Direction.FORWARD);
        flyWheel2.setDirection(DcMotor.Direction.REVERSE);
        //conveyor1       = hwMap.get(DcMotor.class, "conveyor1");

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        flyWheel1.setPower(0);
        flyWheel2.setPower(0);

        // Set run modes
        //reset encoders
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set to run with encoder
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //set zero behavior
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Define and initialize ALL installed servos.
        turretRotator = hwMap.get(Servo.class, "turretRotateServo");
        turretLauncher = hwMap.get(Servo.class, "turretLaunchServo");
        turretElevator  = hwMap.get(Servo.class, "turretElevator");
        turretRotator.setPosition(0); // 0 should be pointing to the right side of the robot (when facing same direction as front)
        turretLauncher.setPosition(0); // 0 should be fully open
        turretElevator.setPosition(0);

        // Define and initialize ALL installed sensors.
        //touch1 = hwMap.touchSensor.get("touch_sensor");
        //color1 = hwMap.colorSensor.get("color1");




        //Initialize IMU hardware map value. PLEASE UPDATE THIS VALUE TO MATCH YOUR CONFIGURATION
        imu = hwMap.get(BNO055IMU.class, "imu");
        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

    }

    ////////////////////////////// file for storing position and heading info after autonomous //////////////////////////////
    public static File positionXFile = AppUtil.getInstance().getSettingsFile("positionX.txt");
    public static File positionYFile = AppUtil.getInstance().getSettingsFile("positionY.txt");
    public static File headingFile = AppUtil.getInstance().getSettingsFile("heading.txt");

    /**
     * method to write the position and heading of the robot to a file (overwriting any data already there)
     * in order to allow that information to persist after, for example, the execution of an autonomous opmode
     * @param position  the position of the robot, in meters, on a coordinate plane as described in comments at the beginning of PositionAndTargetManager.java
     * @param heading   the heading of the robot, in degrees
     */
    public static void writePositionHeading(double[] position, double heading) {
        ReadWriteFile.writeFile(positionXFile, String.valueOf(position[0]));
        ReadWriteFile.writeFile(positionYFile, String.valueOf(position[1]));

        ReadWriteFile.writeFile(headingFile, String.valueOf(heading));
    }

    /**
     * reads information from positionX.txt and positionY.txt to return the x,y position of the robot
     * @return a double array containing the [x,y] coordinates of the robot on the field, in meters
     */
    public static double[] readPosition() {
        return new double[]{Double.parseDouble(ReadWriteFile.readFile(positionXFile).trim()) , Double.parseDouble(ReadWriteFile.readFile(positionYFile).trim()) };
    }

    /**
     * reads information for heading.txt to return the heading of the robot
     * @return the heading of the robot, in degrees
     */
    public static double readHeading() {
        return Double.parseDouble(ReadWriteFile.readFile(headingFile).trim());
    }

    ////////////////////////////// Movement and utility methods //////////////////////////////
    /**
     * Gets the orientation of the robot using the REV IMU
     * @return the angle of the robot
     */
    public double getZAngle(){
        return (-imu.getAngularOrientation().firstAngle);
    }


}
