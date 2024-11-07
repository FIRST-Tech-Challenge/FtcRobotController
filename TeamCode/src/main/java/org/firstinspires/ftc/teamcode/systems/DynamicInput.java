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
        this.mainCtrl = gamepad1;
        this.subCtrl = gamepad2;
        this.mainProfile = mainProfile;
        this.subProfile = subProfile;
        this.mainSettings = mainProfile.mainGamepad;
        this.subSettings = subProfile.subGamepad;
    }

    // Unified output structure
    public static class Combined {
        public DirectionalOutput directional;
        public ConvertedInputs actions;

        public Combined(DirectionalOutput directionalOutput, ConvertedInputs convertedInputs) {
            this.directional = directionalOutput;
            this.actions = convertedInputs;
        }
    }

    // Directional output class for movement and rotation
    public static class DirectionalOutput {
        public double up, right, down, left, rotation, x, y;

        public DirectionalOutput(double up, double right, double down, double left, double rotateRight,
                double rotateLeft) {
            this.up = up;
            this.right = right;
            this.down = down;
            this.left = left;
            this.rotation = rotateRight - rotateLeft;
            this.y = up - down;
            this.x = right - left;
        }
    }

    // Button data structure with justPressed logic
    public static class ConvertedInputs {
        public boolean extendActuator, retractActuator, groundActuator, actuatorBusy;
        public boolean clawRight, clawLeft, wristUp, wristDown;
        public boolean ascendActuatorExtend, ascendActuatorRetract, ascendActuatorChange;
        public boolean justExtendActuator, justRetractActuator, justGroundActuator;
        public double boostAmount, brakeAmount;

        public ConvertedInputs(Gamepad mainCtrl, Settings.DefaultGamepadSettings mainSettings,
                Gamepad subCtrl, Settings.DefaultGamepadSettings subSettings,
                boolean prevExtend, boolean prevRetract, boolean prevGround) {

            this.extendActuator = getButtonState(subCtrl, subSettings.buttonMapping.extendActuator);
            this.retractActuator = getButtonState(subCtrl, subSettings.buttonMapping.retractActuator);
            this.groundActuator = getButtonState(subCtrl, subSettings.buttonMapping.groundActuator);
            this.actuatorBusy = extendActuator || retractActuator || groundActuator;
            this.clawRight = getAxisValue(subCtrl, subSettings.buttonMapping.clawRight) > subSettings.trigger_threshold;
            this.clawLeft = getAxisValue(subCtrl, subSettings.buttonMapping.clawLeft) > subSettings.trigger_threshold;
            this.wristUp = getButtonState(subCtrl, subSettings.buttonMapping.wristUp);
            this.wristDown = getButtonState(subCtrl, subSettings.buttonMapping.wristDown);
            this.ascendActuatorExtend = getButtonState(subCtrl, subSettings.buttonMapping.ascendActuatorExtend);
            this.ascendActuatorRetract = getButtonState(subCtrl, subSettings.buttonMapping.ascendActuatorRetract);
            this.ascendActuatorChange = getButtonState(subCtrl, subSettings.buttonMapping.ascendActuatorChange);
            this.boostAmount = mainSettings.applyBoostCurve(
                    getAxisValue(mainCtrl, mainSettings.buttonMapping.boost));
            this.brakeAmount = mainSettings.applyBoostCurve(
                    getAxisValue(mainCtrl, mainSettings.buttonMapping.brake));

            // Determine if buttons were just pressed
            this.justExtendActuator = extendActuator && !prevExtend;
            this.justRetractActuator = retractActuator && !prevRetract;
            this.justGroundActuator = groundActuator && !prevGround;
        }

        private boolean getButtonState(Gamepad gamepad, Settings.GamepadButton button) {
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

        private double getAxisValue(Gamepad gamepad, Settings.GamepadAxis axis) {
            switch (axis) {
                case LEFT_TRIGGER:
                    return gamepad.left_trigger;
                case RIGHT_TRIGGER:
                    return gamepad.right_trigger;
                default:
                    throw new IllegalArgumentException("Unexpected axis: " + axis);
            }
        }

    }

    // Generates the directional output based on controller input
    public DirectionalOutput directional() {
        double upPower = (mainCtrl.left_stick_y < 0 ? -mainCtrl.left_stick_y : 0)
                + ((mainCtrl.dpad_up ? 1 : 0) * mainSettings.dpad_sensitivity);
        double downPower = (mainCtrl.left_stick_y > 0 ? mainCtrl.left_stick_y : 0)
                + ((mainCtrl.dpad_down ? 1 : 0) * mainSettings.dpad_sensitivity);
        double rightPower = (mainCtrl.left_stick_x > 0 ? mainCtrl.left_stick_x : 0)
                + ((mainCtrl.dpad_right ? 1 : 0) * mainSettings.dpad_sensitivity);
        double leftPower = (mainCtrl.left_stick_x < 0 ? -mainCtrl.left_stick_x : 0)
                + ((mainCtrl.dpad_left ? 1 : 0) * mainSettings.dpad_sensitivity);
        double rotationRight = mainCtrl.right_bumper ? mainSettings.bumper_sensitivity : 0;
        double rotationLeft = mainCtrl.left_bumper ? mainSettings.bumper_sensitivity : 0;

        return new DirectionalOutput(upPower, rightPower, downPower, leftPower, rotationRight, rotationLeft);
    }

    // Generates all inputs data with justPressed tracking
    public ConvertedInputs action() {
        ConvertedInputs convertedInputs = new ConvertedInputs(mainCtrl, mainSettings, subCtrl, subSettings,
                prevExtendActuator, prevRetractActuator, prevGroundActuator);

        // Update previous states for justPressed tracking
        prevExtendActuator = convertedInputs.extendActuator;
        prevRetractActuator = convertedInputs.retractActuator;
        prevGroundActuator = convertedInputs.groundActuator;

        return convertedInputs;
    }

    // Method to switch between different control profiles
    public void switchProfiles(String mainProfileName, String subProfileName) {
        for (Settings.ControllerProfile profile : Settings.AVAILABLE_PROFILES) {
            if (profile.name.equals(mainProfileName)) {
                this.mainProfile = profile;
                this.mainSettings = profile.mainGamepad;
            }
            if (profile.name.equals(subProfileName)) {
                this.subProfile = profile;
                this.subSettings = profile.subGamepad;
            }
        }
    }

}
