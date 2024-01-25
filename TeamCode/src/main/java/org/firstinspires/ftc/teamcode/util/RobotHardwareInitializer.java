package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.RobotOpMode.ERROR;
import static org.firstinspires.ftc.teamcode.RobotOpMode.INFO;
import static org.firstinspires.ftc.teamcode.RobotOpMode.WARNING;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.util.Other.ArrayTypeValue;
import org.firstinspires.ftc.teamcode.util.Other.CRServoTypeValue;
import org.firstinspires.ftc.teamcode.util.Other.DynamicTypeValue;
import org.firstinspires.ftc.teamcode.util.Other.MotorTypeValue;
import org.firstinspires.ftc.teamcode.util.Other.ServoTypeValue;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * The RobotHardwareInitializer abstracts the process of setting the hardware variables in RobotOpMode.
 * Using this class helps to increase readability inside of the RobotOpMode class.
 */
public class RobotHardwareInitializer {

    private static void Error(Exception e, RobotOpMode robot) {
        robot.log("FATAL ERROR", "Could not initialize drive motors: "+e.getMessage());
        robot.sendTelemetryPacket(true);
        robot.terminateOpModeNow();
    }

    private static void Error(Exception e, OpMode opMode) {
        FTCDashboardPackets dbp = new FTCDashboardPackets("RobotHardwareInit");
        dbp.createNewTelePacket();

        dbp.error(e, true, false);
        opMode.terminateOpModeNow();
    }

    public enum DriveMotor {
        LEFT_FRONT,
        RIGHT_FRONT,
        LEFT_BACK,
        RIGHT_BACK,
        ENCODER_LEFT,
        ENCODER_RIGHT,
        ENCODER_BACK
    }

    public enum Other {
        ARM,
        INTAKE,
        LAUNCHER,
        FINGER,
        WRIST
    }

    public enum Intake {
        LEFT,
        RIGHT
    }

    public static final String FRONT_LEFT_DRIVE = "fl_drv";
    public static final String FRONT_RIGHT_DRIVE = "fr_drv";
    public static final String BACK_LEFT_DRIVE = "bl_drv";
    public static final String BACK_RIGHT_DRIVE = "br_drv";

    public static final String LEFT_ENCODER = FRONT_LEFT_DRIVE;
    public static final String RIGHT_ENCODER = FRONT_RIGHT_DRIVE;
    public static final String BACK_ENCODER = BACK_LEFT_DRIVE;

    public static HashMap<DriveMotor, DcMotor> initializeDriveMotors(final HardwareMap hMap, final OpMode opMode) {
        DcMotor leftFrontDrive;
        DcMotor rightFrontDrive;
        DcMotor leftBackDrive;
        DcMotor rightBackDrive;
        try {
            leftFrontDrive = hMap.get(DcMotor.class, FRONT_LEFT_DRIVE);
            rightFrontDrive = hMap.get(DcMotor.class, FRONT_RIGHT_DRIVE);
            leftBackDrive = hMap.get(DcMotor.class, BACK_LEFT_DRIVE);
            rightBackDrive = hMap.get(DcMotor.class, BACK_RIGHT_DRIVE);

            /*leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            Error(e, opMode);
            return null;
        }

        DcMotor encoderLeft;
        DcMotor encoderRight;
        DcMotor encoderBack;

        try {
            encoderLeft = hMap.dcMotor.get(LEFT_ENCODER);
            encoderRight = hMap.dcMotor.get(RIGHT_ENCODER);
            encoderBack = hMap.dcMotor.get(BACK_ENCODER);
        } catch (Exception e) {
            Error(e, opMode);
            return null;
        }

        HashMap<DriveMotor, DcMotor> motorMap = new HashMap<>();

        motorMap.put(DriveMotor.LEFT_FRONT, leftFrontDrive);
        motorMap.put(DriveMotor.RIGHT_FRONT, rightFrontDrive);
        motorMap.put(DriveMotor.LEFT_BACK, leftBackDrive);
        motorMap.put(DriveMotor.RIGHT_BACK, rightBackDrive);
        motorMap.put(DriveMotor.ENCODER_LEFT, encoderLeft);
        motorMap.put(DriveMotor.ENCODER_RIGHT, encoderRight);
        motorMap.put(DriveMotor.ENCODER_BACK, encoderBack);

        return motorMap;
    }

    /** @noinspection rawtypes*/
    public static HashMap<Other, DynamicTypeValue> initializeAllOtherSystems(final OpMode opMode) {
        HashMap<Other, DynamicTypeValue> out = new HashMap<>();
        // Init Arm
        out.put(Other.ARM, new MotorTypeValue(initializeArm(opMode)));

        // Init Wrist
        out.put(Other.WRIST, new ServoTypeValue(initializeWrist(opMode)));

        // Init Finger
        out.put(Other.FINGER, new ServoTypeValue(initializeFinger(opMode)));

        // Init Launcher
        out.put(Other.LAUNCHER, new ServoTypeValue(initializeLauncher(opMode)));

        // Init Intake
        HashMap<Intake, CRServo> tmp = initializeIntake(opMode);
        final CRServo LEFT = tmp.get(Intake.LEFT);
        final CRServo RIGHT = tmp.get(Intake.RIGHT);
        CRServo[] tmp2 = new CRServo[2];
        tmp2[0] = LEFT;
        tmp2[1] = RIGHT;
        out.put(Other.INTAKE, new ArrayTypeValue<>(tmp2));

        return out;
    }

    @Deprecated
    public static void initializeDriveMotors(RobotOpMode robot) {
        try {
            robot.leftFrontDrive = robot.hardwareMap.get(DcMotor.class, "fl_drv");
            robot.rightFrontDrive = robot.hardwareMap.get(DcMotor.class, "fr_drv");
            robot.leftBackDrive = robot.hardwareMap.get(DcMotor.class, "bl_drv");
            robot.rightBackDrive = robot.hardwareMap.get(DcMotor.class, "br_drv");
            /*
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            */
            robot.leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            robot.leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            robot.rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            robot.rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            Error(e, robot);
        }

        try {
            /*
            robot.encoderLeft = new MotorEx(hardwareMap, "encoderLeft");
            robot.encoderRight = new MotorEx(hardwareMap, "encoderRight");
            robot.encoderBack = new MotorEx(hardwareMap, "encoderBack");
             */
            robot.encoderLeft = robot.hardwareMap.dcMotor.get("encoderLeft");
            robot.encoderRight = robot.hardwareMap.dcMotor.get("encoderRight");
            robot.encoderBack = robot.hardwareMap.dcMotor.get("encoderBack");
        } catch (Exception e) {
            Error(e, robot);
        }
    }

    public static DcMotor initializeArm(final OpMode opMode) {
        DcMotor arm = null;
        try {
            arm = opMode.hardwareMap.get(DcMotor.class, "arm");
            arm.setDirection(DcMotorSimple.Direction.FORWARD);
        } catch (Exception e) {
            Error(e, opMode);
        }
        return arm;
    }

    public static HashMap<Intake, CRServo> initializeIntake(final OpMode opMode) {
        CRServo left = null;
        CRServo right = null;

        try {
            left = opMode.hardwareMap.get(CRServo.class, "intakeLeft");
            right = opMode.hardwareMap.get(CRServo.class, "intakeRight");
        } catch (Exception e) {
            Error(e, opMode);
        }

        HashMap<Intake, CRServo> out = new HashMap<>();
        out.put(Intake.LEFT, left);
        out.put(Intake.RIGHT, right);
        return out;
    }

    public static Servo initializeLauncher(final OpMode opMode) {
        try {
            return opMode.hardwareMap.get(Servo.class, "launcher");
        } catch (Exception e) {
            Error(e, opMode);
        }
        return null;
    }

    @Deprecated
    public static void initializeArm(RobotOpMode robot) {
        try {
            robot.armMotor = robot.hardwareMap.get(DcMotor.class, "arm");
            robot.armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            robot.wristServo = robot.hardwareMap.get(Servo.class, "wrist_servo");
            robot.wristServo.setDirection(Servo.Direction.FORWARD);

            // Prepare the arm motors for movement
            robot.log(INFO, "Arm hardware initialized successfully!");
        } catch(Exception e) {
            robot.log(ERROR, "Error initializing arm hardware: "+e.getMessage());
        }
    }

    public static Servo initializeWrist(final OpMode opMode) {
        try {
            Servo servo = opMode.hardwareMap.get(Servo.class, "wrist_servo");
            servo.setDirection(Servo.Direction.FORWARD);
            return servo;
        } catch(Exception e) {
            Error(e, opMode);
        }
        return null;
    }

    public static Servo initializeFinger(final OpMode opMode) {
        try {
            Servo servo = opMode.hardwareMap.get(Servo.class, "finger_servo");
            servo.setDirection(Servo.Direction.FORWARD);
            return servo;
        } catch(Exception e) {
            Error(e, opMode);
        }
        return null;
    }

    @Deprecated
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

    @Deprecated
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

    @Deprecated
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
