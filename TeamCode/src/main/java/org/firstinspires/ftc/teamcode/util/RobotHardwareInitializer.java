package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

    public interface Component<T extends HardwareDevice> {
        T get(HardwareMap map) throws Exception ;
        String getComponentName();
    }

    public enum MotorComponent implements Component<DcMotor> {
        LEFT_FRONT("fl_drv"),
        RIGHT_FRONT("fr_drv"),
        LEFT_BACK("bl_drv"),
        RIGHT_BACK("br_drv"),

        UPPIES("uppies"), // Used to move the pincher and bucket up and down
        EXTENDER("extender"), // Used to move in intake system forward and back
        INTAKE("intake"), // Used to pick up blocks
        ;
        private final String componentName;
        MotorComponent(String componentName) { this.componentName = componentName; }
        @Override public String getComponentName() { return componentName; }
        @Override public DcMotor get(HardwareMap map) { return map.get(DcMotor.class, getComponentName()); }
        public DcMotorEx getEx(HardwareMap map) throws Exception {
            return map.get(DcMotorEx.class, getComponentName());
        }
    }

    public enum ServoComponent implements Component<Servo> {
        FINGER_1("finger1"),
        FINGER_2("finger2"),
        BUCKET_DUMPER("bucket"), // Used to dump the bucket and return to the collecting position
        INTAKE_TILTER("intake_servo"), // Used to tilt the intake system toward the bucket at to the ground
        ;
        private final String componentName;
        ServoComponent(String componentName) { this.componentName = componentName; }
        @Override public String getComponentName() { return componentName; }
        @Override public Servo get(HardwareMap map) { return map.get(Servo.class, getComponentName()); }
        public ServoEx getEx(HardwareMap map, double minAngle, double maxAngle) throws Exception {
            return new SimpleServo(map, getComponentName(), minAngle, maxAngle);
        }
        public ServoEx getEx(HardwareMap map) throws Exception {
            return new SimpleServo(map, getComponentName(), 0, 0);
        }
    }

    public enum EncoderComponent implements Component<DcMotor> {
        ENCODER_LEFT("fl_drv"),
        ENCODER_RIGHT("fr_drv"),
        ENCODER_BACK("br_drv"),
        ;
        private final String componentName;
        EncoderComponent(String componentName) { this.componentName = componentName; }
        @Override public String getComponentName() { return componentName; }
        @Override public DcMotor get(HardwareMap map) { return map.get(DcMotor.class, getComponentName()); }
    }

    /*public enum Component {
        LEFT_FRONT("fl_drv", DcMotor.class),
        RIGHT_FRONT("fr_drv", DcMotor.class),
        LEFT_BACK("bl_drv", DcMotor.class),
        RIGHT_BACK("br_drv", DcMotor.class),
        ;

        public final String componentName;
        public final Class<? extends HardwareDevice> clazz;

        Component(String componentName, Class<? extends HardwareDevice> clazz) {
            this.componentName = componentName;
            this.clazz = clazz;
        }

        public <T extends HardwareDevice> T cast(HardwareDevice device) {
            return clazz
        }
    }*/

    public static final String FRONT_LEFT_DRIVE = "fl_drv";
    public static final String FRONT_RIGHT_DRIVE = "fr_drv";
    public static final String BACK_LEFT_DRIVE = "bl_drv";
    public static final String BACK_RIGHT_DRIVE = "br_drv";

    public static final String LEFT_ENCODER = FRONT_LEFT_DRIVE;
    public static final String RIGHT_ENCODER = FRONT_RIGHT_DRIVE;
    public static final String BACK_ENCODER = BACK_LEFT_DRIVE;

    public static HashMap<Component, DcMotor> initializeDriveMotors(final HardwareMap hMap, final OpMode opMode) {
        DcMotor leftFrontDrive;
        DcMotor rightFrontDrive;
        DcMotor leftBackDrive;
        DcMotor rightBackDrive;
        try {
            leftFrontDrive = MotorComponent.RIGHT_FRONT.get(hMap);
            rightFrontDrive = MotorComponent.RIGHT_FRONT.get(hMap);
            leftBackDrive = MotorComponent.LEFT_BACK.get(hMap);
            rightBackDrive = MotorComponent.RIGHT_BACK.get(hMap);

            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
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
            encoderLeft = hMap.dcMotor.get(EncoderComponent.ENCODER_LEFT.getComponentName());
            encoderRight = hMap.dcMotor.get(EncoderComponent.ENCODER_RIGHT.getComponentName());
            encoderBack = hMap.dcMotor.get(EncoderComponent.ENCODER_BACK.getComponentName());
        } catch (Exception e) {
            Error(e, opMode);
            return null;
        }

        HashMap<Component, DcMotor> motorMap = new HashMap<>();

        motorMap.put(MotorComponent.LEFT_FRONT, leftFrontDrive);
        motorMap.put(MotorComponent.RIGHT_FRONT, rightFrontDrive);
        motorMap.put(MotorComponent.LEFT_BACK, leftBackDrive);
        motorMap.put(MotorComponent.RIGHT_BACK, rightBackDrive);
        motorMap.put(EncoderComponent.ENCODER_LEFT, encoderLeft);
        motorMap.put(EncoderComponent.ENCODER_RIGHT, encoderRight);
        motorMap.put(EncoderComponent.ENCODER_BACK, encoderBack);

        return motorMap;
    }

    /** @noinspection rawtypes*/
    /*public static HashMap<Other, DynamicTypeValue> initializeAllOtherSystems(final OpMode opMode) {
        HashMap<Other, DynamicTypeValue> out = new HashMap<>();

        // Init Color Sensor
        //out.put(Other.COLOR_SENSOR, new ColorSensorTypeValue(initializeColorSensor(opMode)));

        // Init Webcam
        HashMap<Cameras, WebcamName> tmp = initializeCamera(opMode);
        assert tmp != null;
        out.put(Other.WEBCAM, new ArrayTypeValue<>(tmp.values().toArray()));

        return out;
    }*/

    /*public static ColorSensor initializeColorSensor(final OpMode opMode) {
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
    }*/
}
