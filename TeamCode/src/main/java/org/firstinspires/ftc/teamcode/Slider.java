package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Slider extends EncoderMotorOps {
    private Robot robot;
    private Gamepad gamepad;
    private double manual_speed_factor = 1.0;
    static final private double auto_power = 1.0;
    static final private int pos_max = 7800;
    static final private int pos_min = 0;
    private int pos_middle = 3900;
   //private int pos_auton = 1000;

    private int pos_auton = 1000;
    private boolean verbose = false;
    private int motor_ticks = 1425;

    private int rev_ticks = 250;

    public Slider(Robot robot, Gamepad gamepad)
    {
        super(robot, robot.motorSlider, pos_min, pos_max, auto_power, false);
        this.robot = robot;
        this.gamepad = gamepad;
    }
    public void auton() {
        autoOp(pos_auton);
    }

    public void fold() {
        autoOp(pos_min);
    }

    public void operate() {
        autoOpCompletionCheck();
        if (gamepad.dpad_down) {
            // Go to the bottom
            autoOp(pos_min);
        } else if (gamepad.dpad_up) {
            // Goto the middle
            autoOp(pos_middle);
        } else if (gamepad.left_stick_y != 0) {
            manualOp(gamepad.left_stick_y * manual_speed_factor);
        } else {
            manualDefaultStop();
        }
        logUpdate();
    }
}
