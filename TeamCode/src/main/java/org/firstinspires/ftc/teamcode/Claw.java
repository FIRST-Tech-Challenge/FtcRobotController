package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo servoLeft;
    Servo servoRight;
    private boolean closed = true;
    double left_close, left_open;
    double right_close, right_open;
    double left_close_wide, right_close_wide;


    public Claw(Servo servoLeft, double left_close, double left_close_wide, double left_open,  Servo servoRight, double right_close, double right_close_wide, double right_open) {
        this.servoLeft = servoLeft;
        this.left_close = left_close;
        this.left_open = left_open;
        this.servoRight = servoRight;
        this.right_close = right_close;
        this.right_open = right_open;
        this.left_close_wide = left_close_wide;
        this.right_close_wide = right_close_wide;
        servoLeft.setPosition(left_close);
        servoRight.setPosition(right_close);
    }

    public void open()
    {
        if(closed) {
            servoLeft.setPosition(left_open);
            servoRight.setPosition(right_open);
            closed = false;

        }
    }

    public void close_wide()
    {
        if (!closed) {
            servoLeft.setPosition(left_close_wide);
            servoRight.setPosition(right_close_wide);
            closed = true;
        }
    }
    public void close()
    {
        if (!closed) {
            servoLeft.setPosition(left_close);
           servoRight.setPosition(right_close);
            closed = true;
        }
    }
}
