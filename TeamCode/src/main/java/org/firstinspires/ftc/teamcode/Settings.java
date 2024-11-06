package org.firstinspires.ftc.teamcode;

import java.lang.reflect.Field;

/** @noinspection unused */
public class Settings {
    // makes strafing faster or slower
    public static final double strafe_power_coefficient = 1.2;
    public static final double tileLengthFeet = 2;
    public static double ms_needed_to_park = 10000;

    public static class DefaultGamepadSettings {
        public double left_stick_sensitivity = 1.0;
        public double dpad_sensitivity = 0.3;
        public double bumper_sensitivity = 0.8;
        public double trigger_threshold = 0.1;

        // Add button mapping configuration
        public ButtonMapping buttonMapping;

        public DefaultGamepadSettings() {
            this.buttonMapping = new ButtonMapping();
        }
    }

    public static class ButtonMapping {
        // Actuator controls
        public GamepadButton extendActuator = GamepadButton.Y;
        public GamepadButton retractActuator = GamepadButton.X;
        public GamepadButton groundActuator = GamepadButton.B;

        // Claw controls
        public GamepadAxis clawRight = GamepadAxis.RIGHT_TRIGGER;
        public GamepadAxis clawLeft = GamepadAxis.LEFT_TRIGGER;

        // Wrist controls
        public GamepadButton wristUp = GamepadButton.RIGHT_BUMPER;
        public GamepadButton wristDown = GamepadButton.LEFT_BUMPER;

        // Ascend actuator controls
        public GamepadButton ascendActuatorExtend = GamepadButton.DPAD_UP;
        public GamepadButton ascendActuatorRetract = GamepadButton.DPAD_DOWN;
        public GamepadButton ascendActuatorChange = GamepadButton.DPAD_RIGHT;
    }

    public static enum GamepadButton {
        A, B, X, Y,
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
        LEFT_BUMPER, RIGHT_BUMPER
    }

    public static enum GamepadAxis {
        LEFT_TRIGGER, RIGHT_TRIGGER
    }

    // Define preset profiles
    public static class ControlProfiles {
        public static DefaultGamepadSettings boonstra() {
            DefaultGamepadSettings settings = new DefaultGamepadSettings();
            // Customize button mappings for BBoonstra
            settings.buttonMapping.extendActuator = GamepadButton.Y;
            settings.buttonMapping.retractActuator = GamepadButton.A;
            // ... add other customizations
            return settings;
        }

        public static DefaultGamepadSettings israel() {
            DefaultGamepadSettings settings = new DefaultGamepadSettings();
            // Customize button mappings for CIsrael
            settings.buttonMapping.extendActuator = GamepadButton.B;
            settings.buttonMapping.retractActuator = GamepadButton.X;
            // ... add other customizations
            return settings;
        }
    }

    public static String getDisabledFlags() {
        StringBuilder enabledFlags = new StringBuilder();

        Field[] fields = Deploy.class.getFields();

        for (Field field : fields) {
            try {
                if (!field.getBoolean(null)) {
                    enabledFlags.append(field.getName()).append(", ");
                }
            } catch (IllegalAccessException ignored) {
            }
        }

        return enabledFlags.toString();
    }

    public static class Deploy {
        /*
         * Define deployment flags here;
         * An in-progress build with experiment features won't crash
         * and problems will be able to be isolated if you just
         * uncheck dysfunctional deployment flags first.
         * 
         * Working on a new or experimental feature?
         * Flag it so that we can disable it easily!
         */

        // mechanisms
        public static final boolean ARM = true;

        // magic
        public static final boolean ODOMETRY = true;
        public static final boolean DEBUG = true;
        public static final boolean VICTORY = false;

        public static final boolean SKIP_AUTONOMOUS = false;

    }

}
