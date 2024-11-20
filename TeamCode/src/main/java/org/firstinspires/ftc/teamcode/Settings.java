package org.firstinspires.ftc.teamcode;

import java.lang.reflect.Field;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

@Config
/** @noinspection unused */
public class Settings {
    /**
     * Time in milliseconds needed to ensure safe parking -
     * if there is more time than this, the robot will try to score more points.
     */
    public static double ms_needed_to_park = 10000;

    // Movement settings
    @Config
    public static class Movement {
        /**
         * Multiplier applied to strafe movements to compensate for mechanical
         * differences
         */
        public static double strafe_power_coefficient = 1.2;
        /** Standard FTC field tile length in feet */
        public static final double tileLengthFeet = 2;
        /** Default speed for autonomous movements */
        public static double default_autonomous_speed = 0.38;
    }

    // Hardware settings
    @Config
    public static class Hardware {
        /** Encoder counts per full motor revolution */
        public static final double COUNTS_PER_REVOLUTION = 10323.84; // ish? may need to recalculate later
        /** Diameter of the odometry wheels in inches */
        public static final double WHEEL_DIAMETER_INCHES = 3.5;

        // Servo positions
        @Config
        public static class Servo {
            @Config
            public static class Claw {
                /** Values for open and closed positions on the claw */
                public static double RIGHT_OPEN = 0.7;
                public static double RIGHT_CLOSED = 0.4;
                public static double LEFT_OPEN = 0.55;
                public static double LEFT_CLOSED = 0.9; // TODO TUNE
            }

            @Config
            public static class Wrist {
                public static double HORIZONTAL_POSITION = 0.5;
                public static double CHAMBER_POSITION = -0.75;
                public static double BASKET_POSITION = -0.7;
                public static double VERTICAL_POSITION = -0.5;
            }
        }

        @Config
        public static class IDs {
            // Drive motors
            public static final String FRONT_LEFT_MOTOR = "frontLeft";
            public static final String FRONT_RIGHT_MOTOR = "frontRight";
            public static final String REAR_LEFT_MOTOR = "rearLeft";
            public static final String REAR_RIGHT_MOTOR = "rearRight";

            // Arm components
            public static final String EXTENSOR_LEFT = "extensorLeft";
            public static final String EXTENSOR_RIGHT = "extensorRight";
            public static final String LINEAR_ACTUATOR = "linearActuator";
            public static final String GECKO_LEFT = "geckoLeft";
            public static final String GECKO_RIGHT = "geckoRight";
            public static final String WRIST = "wrist";
            public static final String CLAW_LEFT = "clawL";
            public static final String CLAW_RIGHT = "clawR";
            public static final String SHOULDER = "shoulder";
            public static final String ACTUATOR = "linearActuator";
        }

        public static final double SHOULDER_GEAR_RATIO = 2;
        public static final double SHOULDER_TICKS_PER_DEGREE = Hardware.COUNTS_PER_REVOLUTION * SHOULDER_GEAR_RATIO
                / 360.0;
        public static double SHOULDER_POWER = 0.5; // Adjust based on your needs

        @Config
        public static class Extensor {
            // Positions in encoder ticks
            public static int PICKUP = 0;
            public static int HOVER = -20;
            public static int LOW_RUNG = -500;
            public static int HIGH_RUNG = -1000;

            // Motor power settings
            public static double MOVEMENT_POWER = 0.5;
        }

        @Config
        public static class VerticalExtensor {
            // Positions in encoder ticks
            // TODO: TUNE
            public static int PICKUP = 0;
            public static int HOVER = -20;
            public static int LOW_RUNG = -500;
            public static int HIGH_RUNG = -1000;

            // Motor power settings
            public static double MOVEMENT_POWER = 0.5;
        }

        @Config
        public static class HorizontalExtensor {
            // Positions in encoder ticks
            // TODO: TUNE
            public static int COLLAPSED = 0;
            public static int LEVEL_1 = 50;
            public static int LEVEL_2 = 100;
            public static int EXPANDED = 200;

            // Motor power settings
            public static double MOVEMENT_POWER = 0.7;
        }

        @Config
        public static class LinearActuator {
            // Positions in encoder ticks
            public static int MAX = 1000;
            public static int MIN = 0;

            public static double SPEED = 0.5;
        }

        @Config
        public static class Intake {
            public static double SPEED = 0.5;
        }
    }

    // Autonomous settings
    @Config
    public static class Autonomous {
        @Config
        public static class FieldPositions {
            // Starting positions
            public static Vector2d RED_RIGHT_START = new Vector2d(36, -60);
            public static Vector2d RED_LEFT_START = new Vector2d(-36, -60);
            public static Vector2d BLUE_RIGHT_START = new Vector2d(36, 60);
            public static Vector2d BLUE_LEFT_START = new Vector2d(-36, 60);

            // Scoring positions
            public static Vector2d RED_SCORING_POSITION = new Vector2d(24, -36);
            public static Vector2d BLUE_SCORING_POSITION = new Vector2d(24, 36);

            // Human player positions
            public static Vector2d RED_HUMAN_PLAYER = new Vector2d(-36, -60);
            public static Vector2d BLUE_HUMAN_PLAYER = new Vector2d(-36, 60);

            // Parking positions
            public static Vector2d RED_PARKING = new Vector2d(-60, -60);
            public static Vector2d BLUE_PARKING = new Vector2d(-60, 60);
        }

        @Config
        public static class Movement {
            /** Encoder counts for moving forward one unit */
            public static double FWD_ONE_TILE = 100; // TODO tune
            public static double STRAFE_ONE_TILE = 50; // TODO tune
            public static double TURN_NINETY_DEGREES = 50; // TODO tune
            public static int ENCODERS_NEEDED_TO_CORRECT_ODOMETRY = 3;
        }

        @Config
        public static class Timing {
            /** Pause duration after claw operations (milliseconds) */
            public static long CLAW_PAUSE = 500;
            public static long WRIST_PAUSE = 1000;
            public static long EXTENSOR_PAUSE = 2500;
        }

        @Config
        public static class ColorSensor {
            public static int COLOR_THRESHOLD = 500;
            public static int SAMPLE_COUNT = 30;
        }
    }

    // Gamepad settings
    public static class DefaultGamepadSettings {
        /** Sensitivity multiplier for left stick input */
        public double left_stick_sensitivity = 1.0;
        /** Speed for dpad-based absolute movement, from 0 to 1 */
        public double dpad_movement_speed = 0.3;
        public double trigger_threshold = 0.1;

        /** Deadzone for stick inputs to prevent drift */
        public double stick_deadzone = 0.05;

        /** Sensitivity multiplier for right stick input */
        public double right_stick_sensitivity = 1.0;

        /** Bumper rotation speed */
        public double bumper_rotation_speed = 0.8;

        /** Whether to invert Y axis controls */
        public boolean invert_y_axis = false;

        /** Whether to invert X axis controls */
        public boolean invert_x_axis = false;

        /** Whether to use right stick for rotation instead of bumpers */
        public boolean use_right_stick_rotation = false;

        public final ButtonMapping buttonMapping;

        public DefaultGamepadSettings() {
            this.buttonMapping = new ButtonMapping();
        }

        /**
         * Applies a mathematical curve to the boost input to adjust control response
         * 
         * @param input Raw input value between 0 and 1
         * @return Modified input value between 0 and 1
         */
        public double applyBoostCurve(double input) {
            // Default implementation: simple clamp between 0 and 1
            return Math.max(0, Math.min(1, input));
        }
    }

    public static class ButtonMapping {
        // Extensor controls
        public GamepadButton extendExtensor = GamepadButton.B;
        public GamepadButton retractExtensor = GamepadButton.X;
        public final GamepadButton groundExtensor = GamepadButton.A;
        public final GamepadButton ceilingExtensor = GamepadButton.Y;

        // Movement controls
        public final GamepadAxis moveForward = GamepadAxis.LEFT_STICK_Y;
        public final GamepadAxis moveSideways = GamepadAxis.LEFT_STICK_X;
        public final GamepadAxis rotate = GamepadAxis.RIGHT_STICK_X;

        public GamepadButton rotateRight = GamepadButton.RIGHT_BUMPER;
        public GamepadButton rotateLeft = GamepadButton.LEFT_BUMPER;

        // Claw controls
        public final GamepadButton intakeIn = GamepadButton.RIGHT_TRIGGER;
        public final GamepadButton intakeOut = GamepadButton.OPTIONS;
        public final GamepadButton intakeStop = GamepadButton.LEFT_TRIGGER;

        // Wrist controls
        public GamepadButton wristUp = GamepadButton.RIGHT_BUMPER;
        public GamepadButton wristDown = GamepadButton.LEFT_BUMPER;

        // Ascend extensor controls
        public final GamepadButton ascendExtensorExtend = GamepadButton.DPAD_RIGHT;
        public final GamepadButton ascendExtensorRetract = GamepadButton.DPAD_LEFT;
        public final GamepadButton ascendExtensorGround = GamepadButton.DPAD_DOWN;
        public final GamepadButton ascendExtensorCeiling = GamepadButton.DPAD_UP;

        public final GamepadAxis boost = GamepadAxis.RIGHT_TRIGGER;
        public final GamepadAxis brake = GamepadAxis.LEFT_TRIGGER;

        // Single-direction movement controls
        public GamepadButton moveUp = GamepadButton.DPAD_UP;
        public GamepadButton moveDown = GamepadButton.DPAD_DOWN;
        public GamepadButton moveLeft = GamepadButton.DPAD_LEFT;
        public GamepadButton moveRight = GamepadButton.DPAD_RIGHT;

        // Shoulder controls
        public GamepadButton shoulderUp = GamepadButton.LEFT_STICK_BUTTON;
        public GamepadButton shoulderDown = GamepadButton.RIGHT_STICK_BUTTON;

        // Linear Actuator controls
        public final GamepadButton linearActuatorExtend = GamepadButton.Y;
        public final GamepadButton linearActuatorRetract = GamepadButton.A;
    }

    public enum GamepadButton {
        // Face buttons
        A, B, X, Y,

        // D-pad
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,

        // Shoulder buttons
        LEFT_BUMPER, RIGHT_BUMPER,

        // Center buttons
        START, BACK, GUIDE,

        // Stick buttons
        LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON,
        OPTIONS,
        RIGHT_TRIGGER, LEFT_TRIGGER
    }

    public enum GamepadAxis {
        LEFT_TRIGGER, RIGHT_TRIGGER,
        LEFT_STICK_X, LEFT_STICK_Y,
        RIGHT_STICK_X, RIGHT_STICK_Y
    }

    // Deploy flags
    public static class Deploy {
        // Core Mechanisms
        public static final boolean ARM = false;
        public static final boolean LINEAR_ACTUATOR = true;

        // Navigation Systems
        public static final boolean ODOMETRY = true;

        // Development Features
        public static final boolean DEBUG = true;
        public static final boolean SKIP_AUTONOMOUS = false;
        public static final boolean USE_ROADRUNNER = true;

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
        public final String name;
        public final DefaultGamepadSettings mainGamepad;
        public final DefaultGamepadSettings subGamepad;

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
                    dpad_movement_speed = 0.8;
                    bumper_rotation_speed = 0.7;
                }

                @Override
                public double applyBoostCurve(double input) {
                    return BoostCurves.smooth(input);
                }
            },
            new DefaultGamepadSettings() {
                {
                    // Customize sub gamepad settings
                    buttonMapping.extendExtensor = GamepadButton.Y;
                    buttonMapping.retractExtensor = GamepadButton.A;
                    trigger_threshold = 0.2;
                }
            });

    public static final ControllerProfile CISRAEL_PROFILE = new ControllerProfile(
            "cisrael",
            new DefaultGamepadSettings() {
                {
                    dpad_movement_speed = 0.6;
                    bumper_rotation_speed = 0.9;
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

    public static final ControllerProfile RSHARMA_PROFILE = new ControllerProfile(
            "rsharma",
            new DefaultGamepadSettings() {
                {
                    dpad_movement_speed = 0.5;
                    bumper_rotation_speed = 0.9;
                }

                @Override
                public double applyBoostCurve(double input) {
                    return BoostCurves.linear(input);
                }
            },
            new DefaultGamepadSettings() {
                {
                    // Customize sub gamepad settings
                    trigger_threshold = 0.1;
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
