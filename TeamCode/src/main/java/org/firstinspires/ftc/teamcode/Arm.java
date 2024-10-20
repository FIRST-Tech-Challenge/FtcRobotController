package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Gamepad;



public class Arm {
    private Robot robot;
    private Gamepad gamepad;
    static private double pos_sample = 0.4;
    static private double pos_folded = 0.65;
    static private double pos_specimen = 0.55;

    //    static private double pos_whitepixel = 0.215;
    public Arm(Robot robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;
        //arm_pixel();
    }

    public void setPosSample() {
        robot.servoArm.setPosition(pos_sample);
    }


    public void setPosFold() {
        robot.servoArm.setPosition(pos_folded);
    }

    public void setPosSpecimen() {
        robot.servoArm.setPosition(pos_specimen);
    }

}


