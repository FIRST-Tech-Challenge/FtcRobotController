package org.firstinspires.ftc.teamcode.constants;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice.Manufacturer;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Scalar;

import java.lang.reflect.Field;

/**
 * <p>
 *     Each variant represents a type that is supported by the {@link ConstantsLoader} and
 *     {@link ConstantsSaver} classes. Additionally, there is a "catch-all" variant UNSUPPORTED for
 *     any type which is not currently supported.
 * </p>
 * <p>
 *     Currently, the list of supported types are as follows:
 *     <ul>
 *         <li>All primitive types and their wrappers.</li>
 *         <li>All enums present on the {@link DcMotor} (excluding {@link Manufacturer} because
 *             it's read-only and should never be saved or loaded).</li>
 *         <li>All enums present on {@link Servo}</li>
 *         <li>The flavour of Pose2D {@link SparkFunOTOS.Pose2D}</li>
 *     </ul>
 * </p>
 */
enum SupportedType {
    // ---------------------------------------------------------------------------------------------
    // Primitives
    // ---------------------------------------------------------------------------------------------
    FLOAT, DOUBLE, BYTE, SHORT, INTEGER, LONG, BOOLEAN, CHAR,
    // ---------------------------------------------------------------------------------------------
    // Objects
    // ---------------------------------------------------------------------------------------------
    STRING, SERVO_DIRECTION, MOTOR_DIRECTION, ZERO_POWER_BEHAVIOUR,
    SPARK_FUN_POSE_2D, RUN_MODE, SCALAR, LOGO_FACING_DIRECTION, USB_FACING_DIRECTION,
    ANGLE_UNIT, DISTANCE_UNIT,
    UNSUPPORTED;

    /**
     * Converts the supplied field into its equivalent. SupportedType enum variant. This is useful
     * if you want to match a fields supported type, as you can't match on Class<?>.
     * Note that both the primitive type and the wrapper class will return the same value. For
     * example both {@link Float} and float will return SupportedType.FLOAT.
     * @param field The field to return the supported type of
     * @return THe field, as a supported type.
     */
    @NonNull public static SupportedType fieldToType(@NonNull Field field) {
        Class<?> fieldType = field.getType();

        if (fieldType == float.class || fieldType == Float.class)     return FLOAT;
        if (fieldType == double.class || fieldType == Double.class)   return DOUBLE;
        if (fieldType == byte.class || fieldType == Byte.class)       return BYTE;
        if (fieldType == short.class || fieldType == Short.class)     return SHORT;
        if (fieldType == int.class || fieldType == Integer.class)     return INTEGER;
        if (fieldType == long.class || fieldType == Long.class)       return LONG;
        if (fieldType == boolean.class || fieldType == Boolean.class) return BOOLEAN;
        if (fieldType == char.class || fieldType == Character.class)  return CHAR;
        if (fieldType == String.class)                                return STRING;
        if (fieldType == Servo.Direction.class)                       return SERVO_DIRECTION;
        if (fieldType == DcMotorSimple.Direction.class)               return MOTOR_DIRECTION;
        if (fieldType == DcMotor.ZeroPowerBehavior.class)             return ZERO_POWER_BEHAVIOUR;
        if (fieldType == SparkFunOTOS.Pose2D.class)                   return SPARK_FUN_POSE_2D;
        if (fieldType == Scalar.class)                                return SCALAR;
        if (fieldType == LogoFacingDirection.class) 			      return LOGO_FACING_DIRECTION;
        if (fieldType == UsbFacingDirection.class)                    return USB_FACING_DIRECTION;
        if (fieldType == AngleUnit.class)                             return ANGLE_UNIT;
        if (fieldType == DistanceUnit.class)                          return DISTANCE_UNIT;

        return UNSUPPORTED;
    }
}