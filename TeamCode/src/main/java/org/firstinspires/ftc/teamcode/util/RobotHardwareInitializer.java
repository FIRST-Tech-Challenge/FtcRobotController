package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.RobotOpMode.ERROR;
import static org.firstinspires.ftc.teamcode.RobotOpMode.INFO;
import static org.firstinspires.ftc.teamcode.RobotOpMode.WARNING;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotOpMode;

/**
 * The RobotHardwareInitializer abstracts the process of setting the hardware variables in RobotOpMode.
 * Using this class helps to increase readability inside of the RobotOpMode class.
 */
public class RobotHardwareInitializer {

    public static void initializeDriveMotors(RobotOpMode robot) {
        try {
            robot.leftFrontDrive = robot.hardwareMap.get(DcMotor.class, "fl_drv");
            robot.rightFrontDrive = robot.hardwareMap.get(DcMotor.class, "fr_drv");
            robot.leftBackDrive = robot.hardwareMap.get(DcMotor.class, "bl_drv");
            robot.rightBackDrive = robot.hardwareMap.get(DcMotor.class, "br_drv");
        } catch(Exception e) {
            robot.log("FATAL ERROR", "Could not initialize drive motors: "+e.getMessage());
            robot.sendTelemetryPacket(true);
            robot.terminateOpModeNow();
        }
    }

    public static void initializeArm(RobotOpMode robot) {
        try {
            robot.armMotor = robot.hardwareMap.get(DcMotor.class, "arm");
            robot.armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            robot.wristServo = robot.hardwareMap.get(CRServo.class, "wrist_servo");
            robot.wristServo.setDirection(DcMotorSimple.Direction.FORWARD);

            // Prepare the arm motors for movement
            robot.log(INFO, "Arm hardware initialized successfully!");
        } catch(Exception e) {
            robot.log(ERROR, "Error initializing arm hardware: "+e.getMessage());
        }
    }

    public static void initializeIMU(RobotOpMode robot) {
        try {
            robot.imu = robot.hardwareMap.get(BNO055IMU.class, "imu");

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.temperatureUnit = BNO055IMU.TempUnit.CELSIUS;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            robot.imu.initialize(parameters);
            robot.log(INFO, "IMU initialized successfully!");
        } catch (Exception e) {
            robot.log(ERROR, "Error initializing IMU: " + e.getMessage());
        }
    }

    public static void initializeColorSensor(RobotOpMode robot) {
        try {
            robot.colorSensor = robot.hardwareMap.get(RevColorSensorV3.class, "colorSensor");
            if(robot.colorSensor != null) {
                robot.log(INFO, "REV Color Sensor V3 initialized successfully!");
            } else {
                robot.log(WARNING, "REV Color Sensor V3 not detected.");
            }
        } catch(Exception e) {
            robot.log(WARNING, "Error initializing REV Color Sensor V3: "+e.getMessage());
        }
    }

    public static void initializeTouchSensor(RobotOpMode robot) {
        try {
            robot.touchSensor = robot.hardwareMap.get(RevTouchSensor.class, "touchSensor");
            if(robot.touchSensor != null) {
                robot.log(INFO, "REV Touch Sensor initialized successfully!");
            } else {
                robot.log(WARNING, "REV Touch Sensor not detected.");
            }
        } catch(Exception e) {
            robot.log(WARNING, "Error initializing REV Touch Sensor: "+e.getMessage());
        }
    }

    /**
     * Creates the telemetry packet that contains data regarding the results of the initialization process.
     */
    public static void beginInitialization(RobotOpMode robotOpMode) {
        robotOpMode.createTelemetryPacket();
    }

    /**
     * Used to send the telemetry packet that contains data regarding the results of the initialization process
     */
    public static void endInitialization(RobotOpMode robotOpMode) {
        robotOpMode.sendTelemetryPacket(true);
    }
}
