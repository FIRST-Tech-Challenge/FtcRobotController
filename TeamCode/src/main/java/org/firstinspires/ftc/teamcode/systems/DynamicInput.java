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
    private boolean prevExtendExtensor, prevRetractExtensor, prevGroundExtensor, prevCeilingExtensor, prevClawLeft, prevClawRight;

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

        public Movements(Gamepad mainCtrl, Settings.DefaultGamepadSettings mainSettings) {
            // Get mapped axis values with deadzone
            double leftStickY = applyDeadzone(
                    getAxisValue(mainCtrl, mainSettings.buttonMapping.moveForward),
                    mainSettings.stick_deadzone);
            double leftStickX = applyDeadzone(
                    getAxisValue(mainCtrl, mainSettings.buttonMapping.moveSideways),
                    mainSettings.stick_deadzone);
            double rightStickX = applyDeadzone(
                    getAxisValue(mainCtrl, mainSettings.buttonMapping.rotate),
                    mainSettings.stick_deadzone);

            // Apply sensitivities and inversion
            leftStickY *= mainSettings.invert_y_axis ? -1 : 1;
            leftStickX *= mainSettings.invert_x_axis ? -1 : 1;

            double upPower = (leftStickY < 0 ? -leftStickY : 0) * mainSettings.left_stick_sensitivity;
            double downPower = (leftStickY > 0 ? leftStickY : 0) * mainSettings.left_stick_sensitivity;
            double rightPower = (leftStickX > 0 ? leftStickX : 0) * mainSettings.left_stick_sensitivity;
            double leftPower = (leftStickX < 0 ? -leftStickX : 0) * mainSettings.left_stick_sensitivity;

            // Add absolute movement using mapped buttons
            if (getButtonState(mainCtrl, mainSettings.buttonMapping.moveUp))
                upPower = mainSettings.dpad_movement_speed;
            if (getButtonState(mainCtrl, mainSettings.buttonMapping.moveDown))
                downPower = mainSettings.dpad_movement_speed;
            if (getButtonState(mainCtrl, mainSettings.buttonMapping.moveRight))
                rightPower = mainSettings.dpad_movement_speed;
            if (getButtonState(mainCtrl, mainSettings.buttonMapping.moveLeft))
                leftPower = mainSettings.dpad_movement_speed;

            // Handle rotation based on settings
            double rotationRight = 0;
            double rotationLeft = 0;

            if (mainSettings.use_right_stick_rotation) {
                double rotation = rightStickX * mainSettings.right_stick_sensitivity;
                rotationRight = rotation > 0 ? rotation : 0;
                rotationLeft = rotation < 0 ? -rotation : 0;
            } else {
                rotationRight = getButtonState(mainCtrl, mainSettings.buttonMapping.rotateRight)
                        ? mainSettings.bumper_rotation_speed
                        : 0;
                rotationLeft = getButtonState(mainCtrl, mainSettings.buttonMapping.rotateLeft)
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
        public final boolean extendExtensor, retractExtensor, groundExtensor, ceilingExtensor, extensorBusy;
        public final double clawRight, clawLeft;
        public final boolean wristUp, wristDown;
        public final boolean ascendExtensorExtend, ascendExtensorRetract, ascendExtensorGround, ascendExtensorCeiling;
        public final double boostAmount, brakeAmount;

        public Actions(Gamepad mainCtrl, Settings.DefaultGamepadSettings mainSettings,
                Gamepad subCtrl, Settings.DefaultGamepadSettings subSettings) {
            this.extendExtensor = getButtonState(subCtrl, subSettings.buttonMapping.extendExtensor);
            this.retractExtensor = getButtonState(subCtrl, subSettings.buttonMapping.retractExtensor);
            this.groundExtensor = getButtonState(subCtrl, subSettings.buttonMapping.groundExtensor);
            this.ceilingExtensor = getButtonState(subCtrl, subSettings.buttonMapping.ceilingExtensor);
            this.extensorBusy = extendExtensor || retractExtensor || groundExtensor;
            this.clawRight = getAxisValue(subCtrl, subSettings.buttonMapping.clawRight);
            this.clawLeft = getAxisValue(subCtrl, subSettings.buttonMapping.clawLeft);
            this.wristUp = getButtonState(subCtrl, subSettings.buttonMapping.wristUp);
            this.wristDown = getButtonState(subCtrl, subSettings.buttonMapping.wristDown);
            this.ascendExtensorExtend = getButtonState(subCtrl,
                    subSettings.buttonMapping.ascendExtensorExtend);
            this.ascendExtensorRetract = getButtonState(subCtrl,
                    subSettings.buttonMapping.ascendExtensorRetract);
            this.ascendExtensorGround = getButtonState(subCtrl,
                    subSettings.buttonMapping.ascendExtensorGround);
            this.ascendExtensorCeiling = getButtonState(subCtrl,
                    subSettings.buttonMapping.ascendExtensorCeiling);
            this.boostAmount = mainSettings.applyBoostCurve(
                    getAxisValue(mainCtrl, mainSettings.buttonMapping.boost));
            this.brakeAmount = mainSettings.applyBoostCurve(
                    getAxisValue(mainCtrl, mainSettings.buttonMapping.brake));
        }
    }

    public static class ContextualActions extends Actions {
        public final boolean justExtendExtensor, justRetractExtensor, justGroundExtensor, justCeilingExtensor, justClawLeft, justClawRight;
        private final boolean prevExtendExtensor, prevRetractExtensor, prevGroundExtensor, prevCeilingExtensor, prevClawLeft, prevClawRight;
        public final boolean shoulderUp, shoulderDown;

        public ContextualActions(Gamepad mainCtrl, Settings.DefaultGamepadSettings mainSettings,
                Gamepad subCtrl, Settings.DefaultGamepadSettings subSettings,
                boolean prevExtend, boolean prevRetract, boolean prevGround, boolean prevCeiling, boolean prevClawLeft, boolean prevClawRight) {
            super(mainCtrl, mainSettings, subCtrl, subSettings);

            this.prevExtendExtensor = prevExtend;
            this.prevRetractExtensor = prevRetract;
            this.prevGroundExtensor = prevGround;
            this.prevCeilingExtensor = prevCeiling;
            this.prevClawLeft = prevClawLeft;
            this.prevClawRight = prevClawRight;

            this.justExtendExtensor = extendExtensor && !prevExtend;
            this.justRetractExtensor = retractExtensor && !prevRetract;
            this.justGroundExtensor = groundExtensor && !prevGround;
            this.justCeilingExtensor = ceilingExtensor && !prevCeiling;
            this.justClawLeft = clawLeft > 0.5 && !prevClawLeft;
            this.justClawRight = clawRight > 0.5 && !prevClawRight;

            this.shoulderUp = getButtonState(subCtrl, subSettings.buttonMapping.shoulderUp);
            this.shoulderDown = getButtonState(subCtrl, subSettings.buttonMapping.shoulderDown);
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
                prevExtendExtensor, prevRetractExtensor, prevGroundExtensor, prevCeilingExtensor, prevClawLeft, prevClawRight);

        // Update previous states
        prevExtendExtensor = actions.extendExtensor;
        prevRetractExtensor = actions.retractExtensor;
        prevGroundExtensor = actions.groundExtensor;
        prevCeilingExtensor = actions.ceilingExtensor;
        prevClawLeft = actions.clawLeft > 0.5;
        prevClawRight = actions.clawRight > 0.5;

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
            case START:
                return gamepad.start;
            case BACK:
                return gamepad.back;
            case LEFT_STICK_BUTTON:
                return gamepad.left_stick_button;
            case RIGHT_STICK_BUTTON:
                return gamepad.right_stick_button;
            case GUIDE:
                return gamepad.guide;
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
