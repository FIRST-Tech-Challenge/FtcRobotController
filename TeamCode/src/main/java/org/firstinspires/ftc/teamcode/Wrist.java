package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Wrist {
    private Robot robot;
    private Gamepad gamepad;
    static private double pos_sample  = 0.525;

    static private double pos_specimen = 0.45;
    static private double pos_basket = 0.65;
    static private double pos_drop  = 0.35;

    public Wrist(Robot robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;
        //arm_pixel();
    }

    public void setPosSample()
    {
        robot.servoWrist.setPosition(pos_sample);
    }

    public void setPosSpecimen()
    {
        robot.servoWrist.setPosition(pos_specimen);
    }

    public void setPosBasket()
    {
        robot.servoWrist.setPosition(pos_basket);
    }

    public void setPosDrop()
    {
        robot.servoWrist.setPosition(pos_drop);
    }

}
