package com.technototes.library.control.gamepad.old;

import com.qualcomm.robotcore.hardware.Gamepad;
@Deprecated
public class OldCommandGamepad {
    public double stickDeadzone = 0.1;
    public double stickTriggerThreshold = 0.5;
    public Stick leftStick, rightStick;
    public Dpad dpad;
    //suppliers
    public AxisGamepadComponent ltrigger, rtrigger;
    public ButtonGamepadComponent a, b, x, y, start, back, lbump, rbump;
    private Gamepad gamepad;

    public OldCommandGamepad(Gamepad g) {
        gamepad = g;
        setStickSuppliers(gamepad);
        setTriggerSuppliers(gamepad);
        setButtonSuppliers(gamepad);
        setDpadSupplier(gamepad);

    }

    private void setDpadSupplier(Gamepad g) {
        dpad = new Dpad(new ButtonGamepadComponent(() -> g.dpad_up),
                new ButtonGamepadComponent(() -> g.dpad_down),
                new ButtonGamepadComponent(() -> g.dpad_left),
                new ButtonGamepadComponent(() -> g.dpad_right));
    }

    private void setButtonSuppliers(Gamepad g) {
        a = new ButtonGamepadComponent(() -> g.a);
        b = new ButtonGamepadComponent(() -> g.b);
        x = new ButtonGamepadComponent(() -> g.x);
        y = new ButtonGamepadComponent(() -> g.y);
        start = new ButtonGamepadComponent(() -> g.start);
        back = new ButtonGamepadComponent(() -> g.back);
        lbump = new ButtonGamepadComponent(() -> g.left_bumper);
        rbump = new ButtonGamepadComponent(() -> g.right_bumper);
    }

    private void setTriggerSuppliers(Gamepad g) {
        ltrigger = new AxisGamepadComponent(() -> g.left_trigger, stickDeadzone);
        rtrigger = new AxisGamepadComponent(() -> g.right_trigger, stickDeadzone);
    }

    private void setStickSuppliers(Gamepad g) {
        leftStick = new Stick(new AxisGamepadComponent(() -> g.left_stick_x),
                new AxisGamepadComponent(() -> g.left_stick_y),
                new ButtonGamepadComponent(() -> g.left_stick_button));
        rightStick = new Stick(new AxisGamepadComponent(() -> g.right_stick_x),
                new AxisGamepadComponent(() -> g.right_stick_y),
                new ButtonGamepadComponent(() -> g.right_stick_button));
    }

    public boolean getA() {
        return a.getAsBoolean();
    }

    public boolean getB() {
        return b.getAsBoolean();
    }

    public boolean getX() {
        return x.getAsBoolean();
    }

    public boolean getY() {
        return y.getAsBoolean();
    }

    public boolean getStart() {
        return start.getAsBoolean();
    }

    public boolean getBack() {
        return back.getAsBoolean();
    }

    public double getLeftTrigger() {
        return ltrigger.getAsDouble();
    }

    public double getRightTrigger() {
        return rtrigger.getAsDouble();
    }

    public class Stick {
        //supplier
        public ButtonGamepadComponent press;
        public AxisGamepadComponent x, y;

        public Stick(AxisGamepadComponent d1, AxisGamepadComponent d2, ButtonGamepadComponent b) {
            x = d2;
            y = d1;
            press = b;
        }


        public double getXAxis() {
            return x.getAsDouble();
        }

        public double getYAxis() {
            return y.getAsDouble();
        }

        public boolean getPress() {
            return press.getAsBoolean();
        }

        public Dpad getAsDpad() {
            return new Dpad(new ButtonGamepadComponent(() -> y.getAsDouble() > stickTriggerThreshold),
                    new ButtonGamepadComponent(() -> -y.getAsDouble() < stickTriggerThreshold),
                    new ButtonGamepadComponent(() -> -x.getAsDouble() < stickTriggerThreshold),
                    new ButtonGamepadComponent(() -> x.getAsDouble() > stickTriggerThreshold));
        }
    }

    public class Dpad {

        //suppliers
        public ButtonGamepadComponent up, down, left, right;

        public Dpad(ButtonGamepadComponent u, ButtonGamepadComponent d,
                    ButtonGamepadComponent l, ButtonGamepadComponent r) {
            up = u;
            down = d;
            left = l;
            right = r;
        }

        public Stick getAsStick() {
            return new Stick(new AxisGamepadComponent(() -> (up.getAsBoolean() ? 1 : 0) - (down.getAsBoolean() ? 1 : 0)),
                    new AxisGamepadComponent(() -> (right.getAsBoolean() ? 1 : 0) - (left.getAsBoolean() ? 1 : 0)),
                    new ButtonGamepadComponent(() -> false));
        }

        public boolean getUp() {
            return up.getAsBoolean();
        }

        public boolean getDown() {
            return down.getAsBoolean();
        }

        public boolean getLeft() {
            return left.getAsBoolean();
        }

        public boolean getRight() {
            return right.getAsBoolean();
        }
    }
}
