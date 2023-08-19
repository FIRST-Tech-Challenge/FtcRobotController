package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.Utility.AngleUtilities;
import org.firstinspires.ftc.teamcode.hardware.FloatButton;

public class JoyStickButton extends FloatButton {

    public FloatButton x;
    public FloatButton y;

    float rawAngle;
    float angle;

    public JoyStickButton(final String name,
                          final float deadZoneValue) {
        super(name);

        x = new FloatButton(String.format("%s_x", name), deadZoneValue, false);
        y = new FloatButton(String.format("%s_y", name), deadZoneValue, true);
    }

    public void update(final Float xUpdateValue, final Float yUpdateValue, final double updateTime) {

        final float radius = (float) AngleUtilities.getRadius(xUpdateValue, yUpdateValue);
        super.update(radius, updateTime);

        x.update(xUpdateValue, updateTime);
        y.update(yUpdateValue, updateTime);

        // offset by 90 degrees so that forward is angle 0
        this.rawAngle = (float) AngleUtilities.getAngle(y.getRawValue(), -x.getRawValue());

        if (this.isPressed()) {
            this.angle = rawAngle;
        } else {
            // Retain last pressed angle
        }
    }

    public float getRawAngle() {
        return rawAngle;
    }

    public float getAngel() {
        return angle;
    }
}