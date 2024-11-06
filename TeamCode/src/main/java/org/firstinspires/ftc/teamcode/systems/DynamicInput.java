package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Settings;

public class DynamicInput {
    private final Gamepad mainCtrl;
    private final Gamepad subCtrl;
    private Settings.DefaultGamepadSettings mainSettings;
    private Settings.DefaultGamepadSettings subSettings;

    // Track previous button states for justPressed functionality
    private boolean prevExtendActuator, prevRetractActuator, prevGroundActuator;

    public DynamicInput(Gamepad gamepad1, Gamepad gamepad2) {
        this.mainCtrl = gamepad1;
        this.subCtrl = gamepad2;
        this.mainSettings = new Settings.DefaultGamepadSettings();
        this.subSettings = new Settings.DefaultGamepadSettings();
    }

    // Unified output structure
    public static class DynamicInputOutput {
        public DirectionalOutput directionalOutput;
        public ConvertedInputs convertedInputs;

        public DynamicInputOutput(DirectionalOutput directionalOutput, ConvertedInputs convertedInputs) {
            this.directionalOutput = directionalOutput;
            this.convertedInputs = convertedInputs;
        }
    }

    // Directional output class for movement and rotation
    public static class DirectionalOutput {
        public double up, right, down, left, rotation, x, y;

        public DirectionalOutput(double up, double right, double down, double left, double rotateRight, double rotateLeft) {
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

        public ConvertedInputs(Gamepad mainCtrl, Settings.DefaultGamepadSettings mainSettings,
                               Gamepad subCtrl, Settings.DefaultGamepadSettings subSettings,
                               boolean prevExtend, boolean prevRetract, boolean prevGround) {

            this.extendActuator = subCtrl.y;
            this.retractActuator = subCtrl.x;
            this.groundActuator = subCtrl.b;
            this.actuatorBusy = extendActuator || retractActuator || groundActuator;
            this.clawRight = subCtrl.right_trigger > subSettings.trigger_threshold;
            this.clawLeft = subCtrl.left_trigger > subSettings.trigger_threshold;
            this.wristUp = subCtrl.right_bumper;
            this.wristDown = subCtrl.left_bumper;
            this.ascendActuatorExtend = subCtrl.dpad_up;
            this.ascendActuatorRetract = subCtrl.dpad_down;
            this.ascendActuatorChange = subCtrl.dpad_right;

            // Determine if buttons were just pressed
            this.justExtendActuator = extendActuator && !prevExtend;
            this.justRetractActuator = retractActuator && !prevRetract;
            this.justGroundActuator = groundActuator && !prevGround;
        }
    }

    // Generates the directional output based on controller input
    public DirectionalOutput directional() {
        double upPower = (mainCtrl.left_stick_y < 0 ? -mainCtrl.left_stick_y : 0) + ((mainCtrl.dpad_up ? 1 : 0) * mainSettings.dpad_sensitivity);
        double downPower = (mainCtrl.left_stick_y > 0 ? mainCtrl.left_stick_y : 0) + ((mainCtrl.dpad_down ? 1 : 0) * mainSettings.dpad_sensitivity);
        double rightPower = (mainCtrl.left_stick_x > 0 ? mainCtrl.left_stick_x : 0) + ((mainCtrl.dpad_right ? 1 : 0) * mainSettings.dpad_sensitivity);
        double leftPower = (mainCtrl.left_stick_x < 0 ? -mainCtrl.left_stick_x : 0) + ((mainCtrl.dpad_left ? 1 : 0) * mainSettings.dpad_sensitivity);
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
    public void switchProfile(Settings.DefaultGamepadSettings mainSettings, Settings.DefaultGamepadSettings subSettings) {
        this.mainSettings = mainSettings;
        this.subSettings = subSettings;
    }
}
