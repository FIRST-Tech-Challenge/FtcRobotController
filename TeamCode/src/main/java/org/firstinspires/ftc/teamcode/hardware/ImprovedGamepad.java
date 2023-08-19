package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;


public class ImprovedGamepad {

    private final static float DEFAULT_STICK_DEAD_ZONE_VALUE = 0.1f;

    private final Gamepad hardwareGamepad;
    private final ElapsedTime timer;
    private final String name;
    private final float stickDeadZoneValue;

    public FloatButton left_stick_x;
    public FloatButton left_stick_y;
    public FloatButton right_stick_x;
    public FloatButton right_stick_y;
    public BooleanButton dpad_up;
    public BooleanButton dpad_down;
    public BooleanButton dpad_left;
    public BooleanButton dpad_right;
    public BooleanButton a;
    public BooleanButton b;
    public BooleanButton x;
    public BooleanButton y;
    public BooleanButton guide;
    public BooleanButton start;
    public BooleanButton back;
    public BooleanButton left_bumper;
    public BooleanButton right_bumper;
    public BooleanButton left_stick_button;
    public BooleanButton right_stick_button;
    public JoyStickButton left_stick;
    public JoyStickButton right_stick;
    public FloatButton left_trigger;
    public FloatButton right_trigger;


    public double raw_left_stick_radius = 0;
    public double raw_right_stick_radius = 0;

    public double left_stick_radius = 0;
    public double right_stick_radius = 0;

    // angle 0 = pushed forward, angle 90 = pushed left, etc
    public double left_stick_angle = 0;
    public double right_stick_angle = 0;

    public ImprovedGamepad(final Gamepad hardwareGamepad,
                           final ElapsedTime timer,
                           final String name) {
        this(hardwareGamepad, timer, name, DEFAULT_STICK_DEAD_ZONE_VALUE);
    }

    public ImprovedGamepad(final Gamepad hardwareGamepad,
                           final ElapsedTime timer,
                           final String name,
                           final float stickDeadZoneValue) {

        this.hardwareGamepad = hardwareGamepad;
        this.timer = timer;
        this.name = (null != name) ? name : "";
        this.stickDeadZoneValue = stickDeadZoneValue;

        this.right_stick = new JoyStickButton(String.format("%s_right_stick", this.name), stickDeadZoneValue);
        this.left_stick = new JoyStickButton(String.format("%s_left_stick", this.name), stickDeadZoneValue);

        this.left_stick_x = this.left_stick.x;
        this.left_stick_y = this.left_stick.y;
        this.right_stick_x = this.right_stick.x;
        this.right_stick_y = this.right_stick.y;

        this.dpad_up = new BooleanButton(String.format("%s_dpad_up", this.name));
        this.dpad_down = new BooleanButton(String.format("%s_dpad_down", this.name));
        this.dpad_left = new BooleanButton(String.format("%s_dpad_left", this.name));
        this.dpad_right = new BooleanButton(String.format("%s_dpad_right", this.name));
        this.a = new BooleanButton(String.format("%s_a", this.name));
        this.b = new BooleanButton(String.format("%s_b", this.name));
        this.x = new BooleanButton(String.format("%s_x", this.name));
        this.y = new BooleanButton(String.format("%s_y", this.name));
        this.guide = new BooleanButton(String.format("%s_guide", this.name));
        this.start = new BooleanButton(String.format("%s_start", this.name));
        this.back = new BooleanButton(String.format("%s_back", this.name));
        this.left_bumper = new BooleanButton(String.format("%s_left_bumper", this.name));
        this.right_bumper = new BooleanButton(String.format("%s_right_bumper", this.name));
        this.left_stick_button = new BooleanButton(String.format("%s_left_stick_button", this.name));
        this.right_stick_button = new BooleanButton(String.format("%s_right_stick_button", this.name));

        this.left_trigger = new FloatButton(String.format("%s_left_trigger", this.name), 0.25f);
        this.right_trigger = new FloatButton(String.format("%s_right_trigger", this.name), 0.25f);

    }

    public void update() {

        double time = timer.time();

        left_stick.update(hardwareGamepad.left_stick_x, hardwareGamepad.left_stick_y, time);
        right_stick.update(hardwareGamepad.right_stick_x, hardwareGamepad.right_stick_y, time);
        dpad_up.update(hardwareGamepad.dpad_up, time);
        dpad_down.update(hardwareGamepad.dpad_down, time);
        dpad_left.update(hardwareGamepad.dpad_left, time);
        dpad_right.update(hardwareGamepad.dpad_right, time);
        a.update(hardwareGamepad.a, time);
        b.update(hardwareGamepad.b, time);
        x.update(hardwareGamepad.x, time);
        y.update(hardwareGamepad.y, time);
        guide.update(hardwareGamepad.guide, time);
        start.update(hardwareGamepad.start, time);
        back.update(hardwareGamepad.back, time);
        left_bumper.update(hardwareGamepad.left_bumper, time);
        right_bumper.update(hardwareGamepad.right_bumper, time);
        left_stick_button.update(hardwareGamepad.left_stick_button, time);
        right_stick_button.update(hardwareGamepad.right_stick_button, time);
        left_trigger.update(hardwareGamepad.left_trigger, time);
        right_trigger.update(hardwareGamepad.right_trigger, time);


        raw_right_stick_radius = AngleUtilities.getRadius(right_stick_x.getRawValue(), right_stick_y.getRawValue());

        if (raw_right_stick_radius > stickDeadZoneValue) {
            right_stick_radius = (raw_right_stick_radius - stickDeadZoneValue) / (1 - stickDeadZoneValue);
            right_stick_angle = getJoystickAngle(right_stick_x, right_stick_y);
        } else {
            right_stick_radius = 0;
            // else keep last known angle
        }

        raw_left_stick_radius = AngleUtilities.getRadius(left_stick_x.getRawValue(), left_stick_y.getRawValue());

        if (raw_left_stick_radius > stickDeadZoneValue) {
            left_stick_radius = (raw_left_stick_radius - stickDeadZoneValue) / (1 - stickDeadZoneValue);
            left_stick_angle = getJoystickAngle(left_stick_x, left_stick_y);
        } else {
            left_stick_radius = 0;
            // else keep last known angle
        }
    }

    private double getJoystickAngle(FloatButton stick_x, FloatButton stick_y) {
        double angleRad = Math.atan2(stick_y.getRawValue(), stick_x.getRawValue());
        double angleDegrees = AngleUtilities.radiansDegreesTranslation(angleRad);
        // offset by 90 degrees so that forward is angle 0
        return AngleUtilities.getNormalizedAngle(angleDegrees - 90);
    }

    public boolean areButtonsActive(){
        return dpad_left.isPressed() || dpad_down.isPressed() || dpad_up.isPressed() || dpad_right.isPressed() ||
                a.isPressed() || b.isPressed() || x.isPressed() || y.isPressed() ||
                right_bumper.isPressed() || left_bumper.isPressed() || right_trigger.isPressed() ||
                left_trigger.isPressed() || right_stick.isPressed() || left_stick.isPressed();
    }
}