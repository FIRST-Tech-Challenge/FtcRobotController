package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.core.Subsystem;

public class SticksSubsystem extends Subsystem {
    private UpliftRobot robot;
    public Servo stickLeft;
    public Servo stickRight;
    public CRServo sweeperLeft;
    public CRServo sweeperRight;

    public SticksSubsystem(UpliftRobot robot) {
        super(robot);
        this.robot = robot;
        this.stickLeft = robot.stickLeft;
        this.stickRight = robot.stickRight;
        this.sweeperLeft = robot.sweeperLeft;
        this.sweeperRight = robot.sweeperRight;
    }

    @Override
    public void enable() {

    }

    @Override
    public void disable() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void safeDisable() {

    }

    public void dropSticks() {
        // CHANGE THESE VALUES
        stickLeft.setPosition(0.7);
        stickRight.setPosition(1);
    }

    public void raiseSticks() {
        stickLeft.setPosition(1);
        stickRight.setPosition(0.5);
    }

}
