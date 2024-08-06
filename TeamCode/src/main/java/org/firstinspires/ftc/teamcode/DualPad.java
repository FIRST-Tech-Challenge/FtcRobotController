package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class DualPad extends Gamepad {
    static Gamepad emptyPad = new Gamepad();
    int count;
    DualPad previous;
    DualPad gamepad1;
    DualPad gamepad2;
    DualPad shift;

    public void init() {
        count = 0;
        previous = new DualPad();
        previous.gamepad1 = new DualPad();
        previous.gamepad2 = new DualPad();
        previous.shift = new DualPad();
        gamepad1 = new DualPad();
        gamepad2 = new DualPad();
        previous.gamepad1.shift = new DualPad();
        previous.gamepad2.shift = new DualPad();
        gamepad1.shift = new DualPad();
        gamepad2.shift = new DualPad();
        shift = new DualPad();

        // map uninitialized references to objects created above
        // this allows ".shift.gamepad1" as an alias for ".gamepad1.shift",
        // ".gamepad1.previous" as an alias for ".previous.gamepad1", etc.
        // and avoids null pointer dereferences

        previous.previous = previous;

        previous.gamepad1.previous = previous.gamepad1;
        previous.gamepad1.gamepad1 = previous.gamepad1;
        previous.gamepad1.gamepad2 = previous.gamepad2;
        
        previous.gamepad1.shift.previous = previous.gamepad1.shift;
        previous.gamepad1.shift.gamepad1 = previous.gamepad1.shift;
        previous.gamepad1.shift.gamepad2 = previous.gamepad2.shift;
        previous.gamepad1.shift.shift = previous.gamepad1.shift;
        
        previous.gamepad2.previous = previous.gamepad2;
        previous.gamepad2.gamepad1 = previous.gamepad1;
        previous.gamepad2.gamepad2 = previous.gamepad2;
        
        previous.gamepad2.shift.previous = previous.gamepad2.shift;
        previous.gamepad2.shift.gamepad1 = previous.gamepad1.shift;
        previous.gamepad2.shift.gamepad2 = previous.gamepad2.shift;
        previous.gamepad2.shift.shift = previous.gamepad2.shift;
        
        previous.shift.previous = previous.shift;
        previous.shift.gamepad1 = previous.gamepad1.shift;
        previous.shift.gamepad2 = previous.gamepad2.shift;
        previous.shift.shift = previous.shift;

        gamepad1.previous = previous.gamepad1;
        gamepad1.gamepad1 = gamepad1;
        gamepad1.gamepad2 = gamepad2;

        gamepad1.shift.previous = previous.gamepad1.shift;
        gamepad1.shift.gamepad1 = gamepad1.shift;
        gamepad1.shift.gamepad2 = gamepad2.shift;
        gamepad1.shift.shift = gamepad1.shift;
        
        gamepad2.previous = previous.gamepad2;
        gamepad2.gamepad1 = gamepad1;
        gamepad2.gamepad2 = gamepad2;
        
        gamepad2.shift.previous = previous.gamepad2.shift;
        gamepad2.shift.gamepad1 = gamepad1.shift;
        gamepad2.shift.gamepad2 = gamepad2.shift;
        gamepad2.shift.shift = gamepad1.shift;
        
        shift.previous = previous.shift;
        shift.gamepad1 = gamepad1.shift;
        shift.gamepad2 = gamepad2.shift;
        shift.shift = shift;
    }

    public void copyshift(Gamepad gp) {
        if (gp.left_bumper) {
            this.fastcopy(emptyPad);
            shift.fastcopy(gp);
            left_stick_x = gp.left_stick_x;
            left_stick_y = gp.left_stick_y;
            right_stick_x = gp.right_stick_x;
            right_stick_y = gp.right_stick_y;
            this.timestamp = (long)gp.timestamp;
        } else {
            this.fastcopy(gp);
            shift.fastcopy(emptyPad);
            shift.timestamp = (long)gp.timestamp;
        }
    }

    public void merge(Gamepad gp1, Gamepad gp2) {
        a = gp1.a || gp2.a;
        b = gp1.b || gp2.b;
        x = gp1.x || gp2.x;
        y = gp1.y || gp2.y;
        dpad_up = gp1.dpad_up || gp2.dpad_up;
        dpad_down = gp1.dpad_down || gp2.dpad_down;
        dpad_left = gp1.dpad_left || gp2.dpad_left;
        dpad_right = gp1.dpad_right || gp2.dpad_right;
        left_bumper = gp1.left_bumper || gp2.left_bumper;
        right_bumper = gp1.right_bumper || gp2.right_bumper;
        back = gp1.back || gp2.back;
        guide = gp1.guide || gp2.guide;
        start = gp1.start || gp2.start;
        left_stick_button = gp1.left_stick_button || gp2.left_stick_button;
        right_stick_button = gp1.right_stick_button || gp2.right_stick_button;
        left_trigger = gp1.left_trigger;
        right_trigger = gp1.right_trigger;
        left_stick_x = gp1.left_stick_x;
        left_stick_y = gp1.left_stick_y;
        right_stick_x = gp1.right_stick_x;
        right_stick_y = gp1.right_stick_y;
        if (left_trigger == 0) left_trigger = gp2.left_trigger;
        if (right_trigger == 0) right_trigger = gp2.right_trigger;
        if (left_stick_x == 0) left_stick_x = gp2.left_stick_x;
        if (left_stick_y == 0) left_stick_y = gp2.left_stick_y;
        if (right_stick_x == 0) right_stick_x = gp2.right_stick_x;
        if (right_stick_y == 0) right_stick_y = gp2.right_stick_y;
        timestamp = Math.max(gp1.timestamp, gp2.timestamp);
    }

    public void update(Gamepad gp1, Gamepad gp2) {
        if (previous == null || shift == null) init();
        count++;
        previous.fastcopy(this);
        previous.shift.fastcopy(shift);
        previous.gamepad1.fastcopy(gamepad1);
        previous.gamepad1.shift.fastcopy(gamepad1.shift);
        previous.gamepad2.fastcopy(gamepad2);
        previous.gamepad2.shift.fastcopy(gamepad2.shift);

        gamepad1.copyshift(gp1);
        gamepad2.copyshift(gp2);
        this.merge(gamepad1, gamepad2);
        shift.merge(gamepad1.shift, gamepad2.shift);
    }
    
    public void fastcopy(Gamepad gp) {
        a = gp.a;
        b = gp.b;
        x = gp.x;
        y = gp.y;
        dpad_up = gp.dpad_up;
        dpad_down = gp.dpad_down;
        dpad_left = gp.dpad_left;
        dpad_right = gp.dpad_right;
        left_bumper = gp.left_bumper;
        right_bumper = gp.right_bumper;
        back = gp.back;
        guide = gp.guide;
        start = gp.start;
        left_stick_button = gp.left_stick_button;
        right_stick_button = gp.right_stick_button;
        left_trigger = gp.left_trigger;
        right_trigger = gp.right_trigger;
        left_stick_x = gp.left_stick_x;
        left_stick_y = gp.left_stick_y;
        right_stick_x = gp.right_stick_x;
        right_stick_y = gp.right_stick_y;
        id = gp.id;
        timestamp = gp.timestamp;
    }
    
}
