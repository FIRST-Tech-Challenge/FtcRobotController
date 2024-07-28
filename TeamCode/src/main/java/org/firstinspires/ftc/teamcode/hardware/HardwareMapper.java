package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.annotation.Annotation;
import java.lang.reflect.Field;

/**
 * Annotation processor for hardware map things.
 */
public abstract class HardwareMapper {
    /**
     * Represents an annotation processor for hardware devices.
     * Internals are a bit of a mess right now.
     * <p> </p>
     * HardwareName is hardcoded because it provides the value of the field.
     */
    static final class DeviceAnnotation<AnnotationT extends Annotation, TargetT> {
        private final Class<AnnotationT> annotationClass;
        private final Class<TargetT> targetClass;
        private final Action<AnnotationT, TargetT> action;

        @FunctionalInterface
        interface Action<AnnotationT extends Annotation, TargetType> {
            void use(AnnotationT annotation, TargetType target);
        }

        DeviceAnnotation(Class<AnnotationT> annotationClass, Class<TargetT> targetClass, Action<AnnotationT, TargetT> action) {
            this.annotationClass = annotationClass;
            this.targetClass = targetClass;
            this.action = action;
        }

        private boolean check(Field t, Object value) {
            if (!t.isAnnotationPresent(annotationClass)) return false;
            AnnotationT annotation = t.getAnnotation(annotationClass);
            if (annotation == null)
                throw new NullPointerException("Annotation is null! (isAnnotationPresent is true but getAnnotation returned null)");
            if (!targetClass.isAssignableFrom(value.getClass()))
                throw new ClassCastException("Annotation " + annotationClass.getName() + " can only be used on " + targetClass.getName() + " (or subclasses)");
            return true;
        }

        void use(Field t, Object value) {
            if (!check(t, value)) return;
            AnnotationT annotation = t.getAnnotation(annotationClass);
            if (annotation == null)
                throw new NullPointerException("Annotation disappeared between check and use");
            action.use(annotation, targetClass.cast(value));
        }
    }

    static final DeviceAnnotation<Reversed, DcMotorSimple> reversed =
            new DeviceAnnotation<>(
                    Reversed.class,
                    DcMotorSimple.class,
                    (annotation, target) -> target.setDirection(DcMotorSimple.Direction.REVERSE)
            );

    static final DeviceAnnotation<ZeroPower, DcMotor> zeroPower =
            new DeviceAnnotation<>(
                    ZeroPower.class,
                    DcMotor.class,
                    (annotation, target) -> target.setZeroPowerBehavior(annotation.value())
            );

    static final DeviceAnnotation<AutoClearEncoder, DcMotor> autoClearEncoder =
            new DeviceAnnotation<>(
                    AutoClearEncoder.class,
                    DcMotor.class,
                    (annotation, target) -> {
                        DcMotor.RunMode current = target.getMode();
                        target.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        target.setMode(current);
                    }
            );

    public HardwareMapper(HardwareMap map) {
        Field[] fields = this.getClass().getDeclaredFields();
        for (Field field : fields) {
            boolean accessible = field.isAccessible();
            if (!accessible) field.setAccessible(true);
            try {
                Class<?> targetType = field.getType();
                HardwareName annotation = field.getAnnotation(HardwareName.class);
                if (annotation == null) continue;
                Object result = map.tryGet(targetType, annotation.value());

                if (result == null) {
                    throw new RuntimeException("Hardware: '" + field.getName() + "' not found, expected type " + targetType.getName() + " for field " + field.getName() + " in " + this.getClass().getName());
                }
                try {
                    field.set(this, result);
                } catch (IllegalAccessException e) {
                    throw new RuntimeException("Field " + field.getName() + " assign failed: " + result.getClass().getName() + " to " + field.getType().getName());
                }

                reversed.use(field, result);
                zeroPower.use(field, result);
                autoClearEncoder.use(field, result);
            } catch (IllegalArgumentException e) {
                throw new RuntimeException("Field " + field.getName() + " typecast failed");
            } finally {
                // lock the field back if it was locked
                field.setAccessible(accessible);
            }
        }
    }
}
