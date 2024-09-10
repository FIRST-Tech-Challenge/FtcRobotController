package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Settings;


public class DynamicInput {
    private final Gamepad mainCtrl;
    private final Settings.DefaultGamepadSettings mainSettings;
    private final Gamepad subCtrl;
    private final Settings.DefaultGamepadSettings subSettings;

    public DynamicInput(Gamepad gamepad1, Gamepad gamepad2) {
        this.mainCtrl = gamepad1;
        this.subCtrl = gamepad2;
        this.mainSettings = new Settings.DefaultGamepadSettings();
        this.subSettings = new Settings.DefaultGamepadSettings();
    }

    public static class DirectionalOutput {
        public double up;
        public double right;
        public double down;
        public double left;
        public double rotation;
        public double x;
        public double y;

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

    // Class to handle button presses
    public static class ConvertedInputs {
        public boolean extendActuator;
        public boolean retractActuator;
        public boolean groundActuator;
        public boolean actuatorBusy;
        public boolean clawRight;
        public boolean clawLeft;
        public boolean wristUp;
        public boolean wristDown;
        public boolean ascendActuatorExtend;
        public boolean ascendActuatorRetract;
        public boolean ascendActuatorChange;

        public ConvertedInputs(Gamepad mainCtrl, Settings.DefaultGamepadSettings mainSettings,
                               Gamepad subCtrl, Settings.DefaultGamepadSettings subSettings) {
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
        }
    }

    public DirectionalOutput getDirectionalOutput() {
        // Up and Down using left stick Y-axis and D-pad
        double upPower = (mainCtrl.left_stick_y < 0 ? -mainCtrl.left_stick_y : 0) + ((mainCtrl.dpad_up ? 1 : 0) * mainSettings.dpad_sensitivity);
        double downPower = (mainCtrl.left_stick_y > 0 ? mainCtrl.left_stick_y : 0) + ((mainCtrl.dpad_down ? 1 : 0) * mainSettings.dpad_sensitivity);

        // Right and Left using left stick X-axis and D-pad
        double rightPower = (mainCtrl.left_stick_x > 0 ? mainCtrl.left_stick_x : 0) + ((mainCtrl.dpad_right ? 1 : 0) * mainSettings.dpad_sensitivity);
        double leftPower = (mainCtrl.left_stick_x < 0 ? -mainCtrl.left_stick_x : 0) + ((mainCtrl.dpad_left ? 1 : 0) * mainSettings.dpad_sensitivity);

        // Rotation using bumpers
        double rotationRight = mainCtrl.right_bumper ? mainSettings.bumper_sensitivity : 0;
        double rotationLeft = mainCtrl.left_bumper ? mainSettings.bumper_sensitivity : 0;

        return new DirectionalOutput(upPower, rightPower, downPower, leftPower, rotationRight, rotationLeft);
    }

    // Method to get pressed button states
    public ConvertedInputs pressed() {
        return new ConvertedInputs(mainCtrl, mainSettings, subCtrl, subSettings);
    }
}