package org.firstinspires.ftc.teamcode;

import java.lang.reflect.Field;

/** @noinspection unused */
public class Settings {
    // Core timing settings
    public static double ms_needed_to_park = 10000;

    // Movement settings
    public static class Movement {
        public static final double strafe_power_coefficient = 1.2;
        public static final double tileLengthFeet = 2;
        public static final double default_autonomous_speed = 0.38;
        public static final double max_autonomous_speed = 0.6;
    }

    // Hardware settings
    public static class Hardware {
        // Odometry
        public static final double COUNTS_PER_REVOLUTION = 100;
        public static final double WHEEL_DIAMETER_INCHES = 3.5;

        // Servo positions
        public static class Servo {
            public static class Claw {
                public static final double RIGHT_OPEN = 0.35;
                public static final double RIGHT_CLOSED = -0.2;
                public static final double LEFT_OPEN = 1.0;
                public static final double LEFT_CLOSED = 0.55;
            }

            public static class Wrist {
                public static final double BOARD_POSITION = 0.075;
                public static final double TRANSIT_POSITION = 0.5;
                public static final double HORIZONTAL_POSITION = 0.45;
            }
        }
    }

    // Autonomous settings
    public static class Autonomous {
        public static class Movement {
            public static final double FORWARD_COUNTS = 100;
            public static final double BACKWARD_COUNTS = 100;
            public static final double STRAFE_COUNTS = 50;
            public static final double TURN_COUNTS = 50;
        }

        public static class Timing {
            public static final long CLAW_PAUSE = 500;
            public static final long WRIST_PAUSE = 1000;
            public static final long EXTENSOR_PAUSE = 2500;
        }

        public static class ColorSensor {
            public static final int COLOR_THRESHOLD = 500;
            public static final int SAMPLE_COUNT = 30;
        }
    }

    // Gamepad settings
    public static class DefaultGamepadSettings {
        public double left_stick_sensitivity = 1.0;
        public double dpad_sensitivity = 0.3;
        public double bumper_sensitivity = 0.8;
        public double trigger_threshold = 0.1;
        public double default_movement_speed = 0.1; // TODO: Tune

        // Add button mapping configuration
        public ButtonMapping buttonMapping;

        public DefaultGamepadSettings() {
            this.buttonMapping = new ButtonMapping();
        }

        // Add boost curve function
        public double applyBoostCurve(double input) {
            // Default implementation: simple clamp between 0 and 1
            return Math.max(0, Math.min(1, input));
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

        public GamepadAxis boost = GamepadAxis.RIGHT_TRIGGER;
        public GamepadAxis brake = GamepadAxis.LEFT_TRIGGER;
    }

    public static enum GamepadButton {
        // Face buttons
        A, B, X, Y,

        // D-pad
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,

        // Shoulder buttons
        LEFT_BUMPER, RIGHT_BUMPER;

        // Add documentation for each button group
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

    // Deploy flags
    public static class Deploy {
        // Core Mechanisms
        public static final boolean ARM = true;

        // Navigation Systems
        public static final boolean ODOMETRY = true;

        // Development Features
        public static final boolean DEBUG = true;
        public static final boolean SKIP_AUTONOMOUS = false;

        // Special Features
        public static final boolean VICTORY = false;
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

    public static class ControllerProfile {
        public String name;
        public DefaultGamepadSettings mainGamepad;
        public DefaultGamepadSettings subGamepad;

        public ControllerProfile(String name, DefaultGamepadSettings main, DefaultGamepadSettings sub) {
            this.name = name;
            this.mainGamepad = main;
            this.subGamepad = sub;
        }
    }

    public static final ControllerProfile DEFAULT_PROFILE = new ControllerProfile(
            "default",
            new DefaultGamepadSettings(),
            new DefaultGamepadSettings());

    public static final ControllerProfile BBOONSTRA_PROFILE = new ControllerProfile(
            "bboonstra",
            new DefaultGamepadSettings() {
                {
                    // Customize main gamepad settings
                    dpad_sensitivity = 0.8;
                    bumper_sensitivity = 0.7;
                }

                @Override
                public double applyBoostCurve(double input) {
                    return BoostCurves.smooth(input);
                }
            },
            new DefaultGamepadSettings() {
                {
                    // Customize sub gamepad settings
                    buttonMapping.extendActuator = GamepadButton.Y;
                    buttonMapping.retractActuator = GamepadButton.A;
                    trigger_threshold = 0.2;
                }
            });

    public static final ControllerProfile CISRAEL_PROFILE = new ControllerProfile(
            "cisrael",
            new DefaultGamepadSettings() {
                {
                    dpad_sensitivity = 0.6;
                    bumper_sensitivity = 0.9;
                }

                @Override
                public double applyBoostCurve(double input) {
                    return BoostCurves.quadratic(input);
                }
            },
            new DefaultGamepadSettings() {
                {
                    // Customize sub gamepad settings
                    buttonMapping.wristUp = GamepadButton.DPAD_RIGHT;
                    buttonMapping.wristDown = GamepadButton.DPAD_LEFT;
                    trigger_threshold = 0.15;
                }
            });

    public static final ControllerProfile[] AVAILABLE_PROFILES = {
            DEFAULT_PROFILE,
            BBOONSTRA_PROFILE,
            CISRAEL_PROFILE
    };

    // Add this new class after the GamepadAxis enum
    public static class BoostCurves {
        public static double linear(double input) {
            return Math.max(0, Math.min(1, input));
        }

        // Quadratic - slower start, faster end
        public static double quadratic(double input) {
            input = Math.max(0, Math.min(1, input));
            return Math.pow(input, 2);
        }

        // Cubic - even slower start
        public static double cubic(double input) {
            input = Math.max(0, Math.min(1, input));
            return Math.pow(input, 3);
        }

        // Square root - faster start, slower end
        public static double squareRoot(double input) {
            input = Math.max(0, Math.min(1, input));
            return Math.sqrt(input);
        }

        // Sine wave - smooth S-curve
        public static double smooth(double input) {
            input = Math.max(0, Math.min(1, input));
            return (Math.sin((input - 0.5) * Math.PI) + 1) / 2;
        }

        // Step function - binary on/off at threshold
        public static double step(double input, double threshold) {
            return input >= threshold ? 1.0 : 0.0;
        }

        // Exponential - very slow start, very fast end
        public static double exponential(double input) {
            input = Math.max(0, Math.min(1, input));
            return (Math.exp(input * 3) - 1) / (Math.exp(3) - 1);
        }

        // Custom curve generator - allows for fine-tuning
        public static double custom(double input, double power) {
            input = Math.max(0, Math.min(1, input));
            return Math.pow(input, power);
        }
    }
}
