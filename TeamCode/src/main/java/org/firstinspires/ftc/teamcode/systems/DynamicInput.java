package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Settings;

public class DynamicInput {
    private final Gamepad mainCtrl;
    private final Gamepad subCtrl;
    private Settings.DefaultGamepadSettings mainSettings;
    private Settings.DefaultGamepadSettings subSettings;
    private Settings.ControllerProfile mainProfile;
    private Settings.ControllerProfile subProfile;

    // Track previous button states for justPressed functionality
    private boolean prevExtendActuator, prevRetractActuator, prevGroundActuator;

    public DynamicInput(Gamepad gamepad1, Gamepad gamepad2, Settings.ControllerProfile mainProfile,
            Settings.ControllerProfile subProfile) {
        if (gamepad1 == null || gamepad2 == null || mainProfile == null || subProfile == null) {
            throw new IllegalArgumentException("All constructor parameters must be non-null");
        }
        this.mainCtrl = gamepad1;
        this.subCtrl = gamepad2;
        this.mainProfile = mainProfile;
        this.subProfile = subProfile;
        this.mainSettings = mainProfile.mainGamepad;
        this.subSettings = subProfile.subGamepad;
    }

    public static class Movements {
        public final double up, right, down, left, rotation, x, y;
        private final Gamepad mainCtrl;
        private final Settings.DefaultGamepadSettings mainSettings;

        public Movements(Gamepad mainCtrl, Settings.DefaultGamepadSettings mainSettings) {
            this.mainCtrl = mainCtrl;
            this.mainSettings = mainSettings;

            // Move logic from movements() method here
            double leftStickY = applyDeadzone(getAxisValue(mainCtrl, Settings.GamepadAxis.LEFT_STICK_Y),
                    mainSettings.stick_deadzone);
            double leftStickX = applyDeadzone(getAxisValue(mainCtrl, Settings.GamepadAxis.LEFT_STICK_X),
                    mainSettings.stick_deadzone);
            double rightStickX = applyDeadzone(getAxisValue(mainCtrl, Settings.GamepadAxis.RIGHT_STICK_X),
                    mainSettings.stick_deadzone);

            // Apply sensitivities and inversion
            leftStickY *= mainSettings.invert_y_axis ? -1 : 1;
            leftStickX *= mainSettings.invert_x_axis ? -1 : 1;

            double upPower = (leftStickY < 0 ? -leftStickY : 0) * mainSettings.left_stick_sensitivity;
            double downPower = (leftStickY > 0 ? leftStickY : 0) * mainSettings.left_stick_sensitivity;
            double rightPower = (leftStickX > 0 ? leftStickX : 0) * mainSettings.left_stick_sensitivity;
            double leftPower = (leftStickX < 0 ? -leftStickX : 0) * mainSettings.left_stick_sensitivity;

            // Add dpad absolute movement using mapped buttons
            if (getButtonState(mainCtrl, Settings.GamepadButton.DPAD_UP))
                upPower = mainSettings.dpad_movement_speed;
            if (getButtonState(mainCtrl, Settings.GamepadButton.DPAD_DOWN))
                downPower = mainSettings.dpad_movement_speed;
            if (getButtonState(mainCtrl, Settings.GamepadButton.DPAD_RIGHT))
                rightPower = mainSettings.dpad_movement_speed;
            if (getButtonState(mainCtrl, Settings.GamepadButton.DPAD_LEFT))
                leftPower = mainSettings.dpad_movement_speed;

            // Handle rotation based on settings
            double rotationRight = 0;
            double rotationLeft = 0;

            if (mainSettings.use_right_stick_rotation) {
                double rotation = rightStickX * mainSettings.right_stick_sensitivity;
                rotationRight = rotation > 0 ? rotation : 0;
                rotationLeft = rotation < 0 ? -rotation : 0;
            } else {
                rotationRight = getButtonState(mainCtrl, Settings.GamepadButton.RIGHT_BUMPER)
                        ? mainSettings.bumper_rotation_speed
                        : 0;
                rotationLeft = getButtonState(mainCtrl, Settings.GamepadButton.LEFT_BUMPER)
                        ? mainSettings.bumper_rotation_speed
                        : 0;
            }

            // Set final values
            this.up = upPower;
            this.right = rightPower;
            this.down = downPower;
            this.left = leftPower;
            this.rotation = rotationRight - rotationLeft;
            this.y = up - down;
            this.x = right - left;
        }

        private double applyDeadzone(double value, double deadzone) {
            return Math.abs(value) > deadzone ? value : 0;
        }
    }

    public static class Actions {
        public final boolean extendActuator, retractActuator, groundActuator, actuatorBusy;
        public final boolean clawRight, clawLeft, wristUp, wristDown;
        public final boolean ascendActuatorExtend, ascendActuatorRetract, ascendActuatorChange;
        public final double boostAmount, brakeAmount;

        public Actions(Gamepad mainCtrl, Settings.DefaultGamepadSettings mainSettings,
                Gamepad subCtrl, Settings.DefaultGamepadSettings subSettings) {
            this.extendActuator = getButtonState(subCtrl, subSettings.buttonMapping.extendActuator);
            this.retractActuator = getButtonState(subCtrl, subSettings.buttonMapping.retractActuator);
            this.groundActuator = getButtonState(subCtrl, subSettings.buttonMapping.groundActuator);
            this.actuatorBusy = extendActuator || retractActuator || groundActuator;
            this.clawRight = getAxisValue(subCtrl, subSettings.buttonMapping.clawRight) > subSettings.trigger_threshold;
            this.clawLeft = getAxisValue(subCtrl, subSettings.buttonMapping.clawLeft) > subSettings.trigger_threshold;
            this.wristUp = getButtonState(subCtrl, subSettings.buttonMapping.wristUp);
            this.wristDown = getButtonState(subCtrl, subSettings.buttonMapping.wristDown);
            this.ascendActuatorExtend = getButtonState(subCtrl,
                    subSettings.buttonMapping.ascendActuatorExtend);
            this.ascendActuatorRetract = getButtonState(subCtrl,
                    subSettings.buttonMapping.ascendActuatorRetract);
            this.ascendActuatorChange = getButtonState(subCtrl,
                    subSettings.buttonMapping.ascendActuatorChange);
            this.boostAmount = mainSettings.applyBoostCurve(
                    getAxisValue(mainCtrl, mainSettings.buttonMapping.boost));
            this.brakeAmount = mainSettings.applyBoostCurve(
                    getAxisValue(mainCtrl, mainSettings.buttonMapping.brake));
        }
    }

    public static class ContextualActions extends Actions {
        public final boolean justExtendActuator, justRetractActuator, justGroundActuator;
        private final boolean prevExtendActuator, prevRetractActuator, prevGroundActuator;

        public ContextualActions(Gamepad mainCtrl, Settings.DefaultGamepadSettings mainSettings,
                Gamepad subCtrl, Settings.DefaultGamepadSettings subSettings,
                boolean prevExtend, boolean prevRetract, boolean prevGround) {
            super(mainCtrl, mainSettings, subCtrl, subSettings);

            this.prevExtendActuator = prevExtend;
            this.prevRetractActuator = prevRetract;
            this.prevGroundActuator = prevGround;

            this.justExtendActuator = extendActuator && !prevExtend;
            this.justRetractActuator = retractActuator && !prevRetract;
            this.justGroundActuator = groundActuator && !prevGround;
        }
    }

    public Movements getMovements() {
        return new Movements(mainCtrl, mainSettings);
    }

    public Actions getActions() {
        return new Actions(mainCtrl, mainSettings, subCtrl, subSettings);
    }

    public ContextualActions getContextualActions() {
        ContextualActions actions = new ContextualActions(mainCtrl, mainSettings, subCtrl, subSettings,
                prevExtendActuator, prevRetractActuator, prevGroundActuator);

        // Update previous states
        prevExtendActuator = actions.extendActuator;
        prevRetractActuator = actions.retractActuator;
        prevGroundActuator = actions.groundActuator;

        return actions;
    }

    // Method to switch between different control profiles
    public void switchProfiles(String mainProfileName, String subProfileName) {
        boolean mainFound = false, subFound = false;
        for (Settings.ControllerProfile profile : Settings.AVAILABLE_PROFILES) {
            if (profile.name.equals(mainProfileName)) {
                this.mainProfile = profile;
                this.mainSettings = profile.mainGamepad;
                mainFound = true;
            }
            if (profile.name.equals(subProfileName)) {
                this.subProfile = profile;
                this.subSettings = profile.subGamepad;
                subFound = true;
            }
        }
        if (!mainFound || !subFound) {
            throw new IllegalArgumentException("Invalid profile name(s)");
        }
    }

    private static boolean getButtonState(Gamepad gamepad, Settings.GamepadButton button) {
        switch (button) {
            case A:
                return gamepad.a;
            case B:
                return gamepad.b;
            case X:
                return gamepad.x;
            case Y:
                return gamepad.y;
            case DPAD_UP:
                return gamepad.dpad_up;
            case DPAD_DOWN:
                return gamepad.dpad_down;
            case DPAD_LEFT:
                return gamepad.dpad_left;
            case DPAD_RIGHT:
                return gamepad.dpad_right;
            case LEFT_BUMPER:
                return gamepad.left_bumper;
            case RIGHT_BUMPER:
                return gamepad.right_bumper;
            default:
                throw new IllegalArgumentException("Unexpected button: " + button);
        }
    }

    private static double getAxisValue(Gamepad gamepad, Settings.GamepadAxis axis) {
        switch (axis) {
            case LEFT_TRIGGER:
                return gamepad.left_trigger;
            case RIGHT_TRIGGER:
                return gamepad.right_trigger;
            case LEFT_STICK_X:
                return gamepad.left_stick_x;
            case LEFT_STICK_Y:
                return gamepad.left_stick_y;
            case RIGHT_STICK_X:
                return gamepad.right_stick_x;
            case RIGHT_STICK_Y:
                return gamepad.right_stick_y;
            default:
                throw new IllegalArgumentException("Unexpected axis: " + axis);
        }
    }
}
