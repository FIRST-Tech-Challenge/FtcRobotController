package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Other.ArrayTypeValue;
import org.firstinspires.ftc.teamcode.util.Other.DynamicTypeValue;
import org.firstinspires.ftc.teamcode.util.Other.MotorTypeValue;
import org.firstinspires.ftc.teamcode.util.Other.ServoTypeValue;

import java.util.HashMap;

/**
 * The RobotHardwareInitializer abstracts the process of setting the hardware variables in RobotOpMode.
 * Using this class helps to increase readability inside of the RobotOpMode class.
 */
public class RobotHardwareInitializer {
    private static void Error(Exception e, OpMode opMode) {
        FTCDashboardPackets dbp = new FTCDashboardPackets("RobotHardwareInit");
        dbp.createNewTelePacket();

        dbp.error(e, true, false);
        opMode.terminateOpModeNow();
    }

    public final static float MIN_POWER = 0;

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
        COLOR_SENSOR,
        WEBCAM,
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

        // Init Color Sensor
        //out.put(Other.COLOR_SENSOR, new ColorSensorTypeValue(initializeColorSensor(opMode)));

        // Init Webcam
        HashMap<Cameras, WebcamName> tmp = initializeCamera(opMode);
        assert tmp != null;
        out.put(Other.WEBCAM, new ArrayTypeValue<>(tmp.values().toArray()));

        return out;
    }

    public enum Arm {
        BUCKET_ARM_MOTOR,
        ARM2,
        ARM3,
        BUCKET_SERVO,
    }

    public static HashMap<Arm, DynamicTypeValue> initializeArm(final OpMode opMode) {
        DcMotor arm = null;
        DcMotor arm2 = null;
        DcMotor arm3 = null;
        Servo bucket_servo = null;
        try {
            arm = opMode.hardwareMap.get(DcMotor.class, "arm1");
            arm.setDirection(DcMotorSimple.Direction.FORWARD);

            arm2 = opMode.hardwareMap.get(DcMotor.class, "arm2");
            arm2.setDirection(DcMotorSimple.Direction.FORWARD);

            arm3 = opMode.hardwareMap.get(DcMotor.class, "arm3");
            arm3.setDirection(DcMotorSimple.Direction.FORWARD);

            bucket_servo = opMode.hardwareMap.get(Servo.class, "bucket_servo");
        } catch (Exception e) {
            Error(e, opMode);
        }

        HashMap<Arm, DynamicTypeValue> out = new HashMap<>();
        out.put(Arm.BUCKET_ARM_MOTOR, new MotorTypeValue(arm));
        out.put(Arm.ARM2, new MotorTypeValue(arm2));
        out.put(Arm.ARM3, new MotorTypeValue(arm3));
        out.put(Arm.BUCKET_SERVO, new ServoTypeValue(bucket_servo));

        return out;
    }

    public static ColorSensor initializeColorSensor(final OpMode opMode) {
        try {
            return opMode.hardwareMap.get(ColorSensor.class, "color_sensor");
        } catch(Exception e) {
            Error(e, opMode);
        }
        return null;
    }

    public enum Cameras {
        CAM1,
        CAM2
    }

    public static HashMap<Cameras, WebcamName> initializeCamera(final OpMode opMode) {
        HashMap<Cameras, WebcamName> out = new HashMap<>();
        try {
            out.put(Cameras.CAM1, opMode.hardwareMap.get(WebcamName.class, "Webcam 1"));
            out.put(Cameras.CAM2, opMode.hardwareMap.get(WebcamName.class, "Webcam 2"));
            return out;
        } catch(Exception e) {
            Error(e, opMode);
        }
        return null;
    }
}
