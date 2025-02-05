package org.firstinspires.ftc.teamcode.constants;


import static org.firstinspires.ftc.teamcode.constants.Constants.FileConstants.*;

import androidx.annotation.NonNull;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.*;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.exceptions.MalformedPropertyException;
import org.firstinspires.ftc.teamcode.constants.exceptions.NoAssociatedConstantsFileException;
import org.firstinspires.ftc.teamcode.utility.Debugger;
import org.opencv.core.Scalar;

import java.io.*;
import java.lang.reflect.*;
import java.util.*;
import java.util.stream.*;

public final class ConstantsLoader {
    private final boolean debug;

    @NonNull private final Debugger debugger;

    /**
     * Creates a new ConstantsLoader with debugging disabled. To enable debugging, pass the opModes
     * telemetry object to the constructor.
     */
    public ConstantsLoader() {
        this.debugger = new Debugger(null);
        this.debug    = false;
    }

    /**
     * Creates a new constants loader with debugging enabled.
     * @param telemetry The telemetry to display debug information on
     */
    public ConstantsLoader(@NonNull Telemetry telemetry) {
        this.debug    = true;
        this.debugger = new Debugger(telemetry);
    }

    @NonNull private List<File> loadConstantsFiles() {
        File constantsDirectory = new File(SD_CARD_PATH);

        if (constantsDirectory.isFile()) {
            String issue = "Failed To Load Constants File Directory"
                    + "\nReason: Conflicting File Constants in OnBot Java Directory."
                    + "\nHelp: Remove Directory Named \"Constants\"";

            debugger.addMessage(issue);

            return new ArrayList<>();
        }

        File[] constantsDirectoryFiles = constantsDirectory.listFiles();

        if (constantsDirectoryFiles == null) {
            debugger.addMessage("No Files Found In Constants Director");
            return new ArrayList<>();
        }

        List<File> textFiles = Arrays.stream(constantsDirectoryFiles)
                .filter(this::isTextFile)
                .collect(Collectors.toList());

        if (textFiles.isEmpty()) debugger.addMessage("No Text Files Found");

        return textFiles;
    }

    private @NonNull Class<?> matchClassToFile(
            @NonNull String fileName
    ) throws NoAssociatedConstantsFileException {
        for (Class<?> clazz : Constants.class.getClasses()) {
            if (!Modifier.isStatic(clazz.getModifiers())) continue;

            if (clazz.getSimpleName().equals(fileName)) return clazz;
        }
        String issue = "Failed To Match Constants Class With Name: " + fileName
                + "\nNote: Non-Static Nested Classes Are Skipped";
        debugger.addMessage(issue);

        throw new NoAssociatedConstantsFileException(fileName);
    }

    /**
     * <p>
     *  Attempts to overwrite all of the values located in {@link Constants} with values found in
     *  corresponding text files located at the following path:
     * </p>
     * <p>
     *     /sdcard/FIRST/java/src/org/firstinspires/ftc/team26396/Constants
     * </p>
     * <p>
     *     By default all errors are silently ignored. To enable debug mode, pass the opModes
     *     telemetry object into the constructor of the ConstantsLoader class.
     * </p>
     */
    public void load() {
        for (File file : loadConstantsFiles()) {
            String fileName = file.getName();

            if (!isTextFile(file)) {
                String issue = "Skipped File " + fileName
                        + "\nReason: Not A Text File";
                debugger.addMessage(issue);
                continue;
            }

            try {
                loadClass(matchClassToFile(stripFileExtension(fileName)), fileName);
            } catch (NoAssociatedConstantsFileException exception) {
                String issue = "Skipped File"
                        + "\nReason: No Constants File Associated With File Name "
                        + exception.fileName;
                debugger.addMessage(issue);
            }
        }

        if (debug) debugger.displayAll();
    }

    private void loadClass(@NonNull Class<?> clazz, @NonNull String fileName) {
        Properties properties = new Properties();

        try {
            properties.load(new FileInputStream(SD_CARD_PATH + fileName));
            for (Field field : clazz.getFields()) {
                try {
                    loadField(field, properties);
                } catch (MalformedPropertyException exception) {
                    debugger.addMalformedPropertyException(exception);
                }
            }
        } catch (IOException ioException) {
            String issue = "Failed To Load Constants File " + fileName
                    + "\nReason: " + ioException.getMessage();
            debugger.addMessage(issue);
        }
    }

    private void loadField(
            @NonNull Field field,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String fieldName = field.getName();

        if (!properties.containsKey(fieldName) || !isLoadable(field)) return;

        try {
            switch (SupportedType.fieldToType(field)) {
                case FLOAT:
                    float floatValue = Float.parseFloat(properties.getProperty(fieldName));
                    field.setFloat(fieldName, floatValue);
                    break;
                case DOUBLE:
                    double doubleValue = Double.parseDouble(properties.getProperty(fieldName));
                    field.setDouble(fieldName, doubleValue);
                    break;
                case BYTE:
                    byte byteValue = Byte.parseByte(properties.getProperty(fieldName));
                    field.setByte(fieldName, byteValue);
                    break;
                case SHORT:
                    short shortValue = Short.parseShort(properties.getProperty(fieldName));
                    field.setShort(fieldName, shortValue);
                    break;
                case INTEGER:
                    int intValue = Integer.parseInt(properties.getProperty(fieldName));
                    field.setInt(fieldName, intValue);
                    break;
                case LONG:
                    long longValue = Long.parseLong(properties.getProperty(fieldName));
                    field.setLong(fieldName, longValue);
                    break;
                case BOOLEAN:
                    boolean booleanValue = Boolean.parseBoolean(properties.getProperty(fieldName));
                    field.setBoolean(field, booleanValue);
                    break;
                case CHAR:
                    field.setChar(field, properties.getProperty(fieldName).toCharArray()[0]);
                case STRING:
                    field.set(fieldName, properties.getProperty(fieldName));
                    break;
                case SERVO_DIRECTION:
                    field.set(fieldName, loadServoDirection(fieldName, properties));
                    break;
                case MOTOR_DIRECTION:
                    field.set(fieldName, loadMotorDirection(fieldName, properties));
                    break;
                case ZERO_POWER_BEHAVIOUR:
                    field.set(fieldName, loadZeroPowerBehaviour(fieldName, properties));
                    break;
                case SPARK_FUN_POSE_2D:
                    field.set(fieldName, loadSparkFunPose2D(fieldName, properties));
                    break;
                case RUN_MODE:
                    field.set(fieldName, loadRunMode(fieldName, properties));
                    break;
                case SCALAR:
                    field.set(fieldName, loadScalar(fieldName, properties));
                    break;
                case LOGO_FACING_DIRECTION:
                    field.set(fieldName, loadLogoFacingDirection(fieldName, properties));
                    break;
                case USB_FACING_DIRECTION:
                    field.set(fieldName, loadUsbFacingDirection(fieldName, properties));
                    break;
                case ANGLE_UNIT:
                    field.set(fieldName, loadAngleUnit(fieldName, properties));
                    break;
                case DISTANCE_UNIT:
                    field.set(fieldName, loadDistanceUnit(fieldName, properties));
                    break;
                case UNSUPPORTED:
                    String issue = "Failed To Load Field " + fieldName
                            + "\nReason: Field Type Not Supported";
                    debugger.addMessage(issue);
                    break;
            }
        } catch (IllegalAccessException ignored) {
            // This is prevented by the isLoadable check at the beginning of the function.
        }
    }

    private boolean isTextFile(@NonNull File file) {
        String fileName = file.getName();
        return fileName.substring(fileName.lastIndexOf('.') + 1).equals("txt");
    }

    @NonNull private String stripFileExtension(@NonNull String fileName) {
        return fileName.substring(0, fileName.lastIndexOf('.'));
    }

    @NonNull private Servo.Direction loadServoDirection(
            @NonNull String key,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String servoDirectionString = properties.getProperty(key);

        switch (servoDirectionString.toLowerCase()) {
            case "forward":
                return Servo.Direction.FORWARD;
            case "reverse":
            case "backwards":
                return Servo.Direction.REVERSE;
            default:
                throw new MalformedPropertyException(
                        key,
                        "Failed To Parse Direction",
                        servoDirectionString);
        }
    }

    @NonNull private DcMotorSimple.Direction loadMotorDirection(
            @NonNull String key,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String motorDirectionString = properties.getProperty(key);

        switch (motorDirectionString.toLowerCase()) {
            case "forward":
                return DcMotor.Direction.FORWARD;
            case "reverse":
            case "backwards":
                return DcMotorSimple.Direction.REVERSE;
            default:
                throw new MalformedPropertyException(
                        key,
                        "Failed To Parse Direction",
                        motorDirectionString
                );
        }
    }

    @NonNull private DcMotor.ZeroPowerBehavior loadZeroPowerBehaviour(
            @NonNull String key,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String zeroPowerBehaviorString = properties.getProperty(key);

        switch (zeroPowerBehaviorString.toLowerCase()) {
            case "float":
                return DcMotor.ZeroPowerBehavior.FLOAT;
            case "brake":
                return DcMotor.ZeroPowerBehavior.BRAKE;
            case "unknown":
                return DcMotor.ZeroPowerBehavior.UNKNOWN;
            default:
                throw new MalformedPropertyException(
                        key,
                        "Failed To Parse ZeroPowerBehavior",
                        zeroPowerBehaviorString
                );
        }
    }

    @NonNull private DcMotor.RunMode loadRunMode(
            @NonNull String key,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String runModeString = properties.getProperty(key);

        switch (runModeString.toLowerCase()) {
            case "run_to_position":
            case "runtoposition":
                return DcMotor.RunMode.RUN_TO_POSITION;
            case "run_using_encoders":
            case "runusingencoders":
                return DcMotor.RunMode.RUN_USING_ENCODER;
            case "run_without_encoders":
            case "runwithoutencoders":
                return DcMotor.RunMode.RUN_WITHOUT_ENCODER;
            case "stop_and_reset_encoders":
            case "stopandresetencoders":
                return DcMotor.RunMode.STOP_AND_RESET_ENCODER;
            default:
                throw new MalformedPropertyException(
                        key,
                        "Failed To Parse As RunMode",
                        runModeString
                );
        }
    }

    @NonNull private UsbFacingDirection loadUsbFacingDirection(
            @NonNull String key,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String USBFacingDirectionString = properties.getProperty(key);

        switch (USBFacingDirectionString.toLowerCase()) {
            case "up":
                return UsbFacingDirection.UP;
            case "down":
                return UsbFacingDirection.DOWN;
            case "left":
                return UsbFacingDirection.LEFT;
            case "right":
                return UsbFacingDirection.RIGHT;
            case "backwards":
                return UsbFacingDirection.BACKWARD;
            case "forwards":
                return UsbFacingDirection.FORWARD;
            default:
                throw new MalformedPropertyException(
                        key,
                        "Failed To Parse As UsbFacingDirection",
                        USBFacingDirectionString
                );
        }
    }

    @NonNull private LogoFacingDirection loadLogoFacingDirection(
            @NonNull String key,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String logoFacingString = properties.getProperty(key);

        switch (logoFacingString.toLowerCase()) {
            case "up":
                return LogoFacingDirection.UP;
            case "down":
                return LogoFacingDirection.DOWN;
            case "left":
                return LogoFacingDirection.LEFT;
            case "right":
                return LogoFacingDirection.RIGHT;
            case "forwards":
                return LogoFacingDirection.FORWARD;
            case "backwards":
                return LogoFacingDirection.BACKWARD;
            default:
                throw new MalformedPropertyException(
                        key,
                        "Failed To Parse As LogoFacingDirection",
                        logoFacingString
                );
        }
    }

    @NonNull private SparkFunOTOS.Pose2D loadSparkFunPose2D(
            @NonNull String key,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String poseString = properties.getProperty(key);

        try {
            double[] poseValues = parseThreePartValue(poseString);
            return new SparkFunOTOS.Pose2D(poseValues[0], poseValues[1], poseValues[2]);
        } catch (NumberFormatException numberFormatException) {
            throw new MalformedPropertyException(
                    key,
                    numberFormatException.getMessage(),
                    poseString
            );
        }
    }

    @NonNull private Scalar loadScalar(
            @NonNull String key,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String scalarString = properties.getProperty(key);

        double[] scalarValues;

        try {
            scalarValues = parseThreePartValue(scalarString);
        } catch (NumberFormatException numberFormatException) {
            throw new MalformedPropertyException(
                    key,
                    numberFormatException.getMessage(),
                    scalarString
            );
        }

        return new Scalar((int) scalarValues[0], (int) scalarValues[1], (int) scalarValues[2]);
    }

    @NonNull private AngleUnit loadAngleUnit(
            @NonNull String key,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String angleUnitString = properties.getProperty(key);

        switch (angleUnitString.toLowerCase()) {
            case "radians":
            case "rad":
                return AngleUnit.RADIANS;
            case "degrees":
            case "deg":
                return AngleUnit.DEGREES;
            default:
                throw new MalformedPropertyException(
                        key,
                        "Cannot Parse As AngleUnit",
                        angleUnitString
                );
        }
    }

    @NonNull private DistanceUnit loadDistanceUnit(
            @NonNull String key,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String distanceUnitString = properties.getProperty(key);

        switch (distanceUnitString.toLowerCase()) {
            case "cm":
            case "centimeter":
                return DistanceUnit.CM;
            case "m":
            case "meter":
                return DistanceUnit.METER;
            case "in":
            case "inches":
                return DistanceUnit.INCH;
            case "mm":
            case "millimeter":
                return DistanceUnit.MM;
            default:
                throw new MalformedPropertyException(
                        key,
                        "Failed To Parse As DistanceUnit",
                        distanceUnitString
                );
        }
    }

    @NonNull private double[] parseThreePartValue(
            @NonNull String value
    ) throws NumberFormatException {
        int indexOfOpenParentheses   = value.indexOf('(');
        int indexOfClosedParentheses = value.indexOf(')');
        int indexOfFirstComma        = value.indexOf(',');
        int indexOfSecondComma       = value.indexOf(',', indexOfFirstComma + 1);

        String valueOneString   = value.substring(indexOfOpenParentheses + 1, indexOfFirstComma);
        String valueTwoString   = value.substring(indexOfFirstComma + 1, indexOfSecondComma + 1);
        String valueThreeString = value.substring(indexOfSecondComma + 1, indexOfClosedParentheses);

        return new double[]{
                Double.parseDouble(valueOneString),
                Double.parseDouble(valueTwoString),
                Double.parseDouble(valueThreeString)
        };
    }

    private boolean isLoadable(@NonNull Field field) {
        int modifiers    = field.getModifiers();
        String fieldName = field.getName();

        boolean fieldIsPublic = Modifier.isPublic(modifiers);
        boolean fieldIsStatic = Modifier.isStatic(modifiers);
        boolean fieldIsFinal  = Modifier.isFinal(modifiers);

        if (!fieldIsPublic || !fieldIsStatic || fieldIsFinal) {
            String message = "Field " + fieldName + " Cannot Be Loaded";

            if (!fieldIsPublic) message += "\nReason: Field Is Not Public";
            if (!fieldIsStatic) message += "\nReason: Field Is Not Static";
            if (fieldIsFinal)   message += "\nReason: Field Is Final";

            debugger.addMessage(message);
            return false;
        }
        return true;
    }
}